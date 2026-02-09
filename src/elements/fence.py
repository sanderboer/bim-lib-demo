import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
from typing import List


# ---------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------


def _coerce_curve(curve) -> rg.Curve:
    crv = rs.coercecurve(curve)
    if not crv:
        raise TypeError("guide must be a Curve")
    return crv


def _frame_at(crv: rg.Curve, t: float) -> rg.Plane:
    """
    Stable frame along curve:
    X = tangent
    Y = left
    Z = up
    """
    ok, u = crv.NormalizedLengthParameter(t)
    if not ok:
        d = crv.Domain
        u = d.T0 + (d.T1 - d.T0) * t

    pt = crv.PointAt(u)

    x = crv.TangentAt(u)
    if x.IsTiny():
        x = rg.Vector3d.XAxis
    x.Unitize()

    z = rg.Vector3d.ZAxis
    y = rg.Vector3d.CrossProduct(z, x)
    if y.IsTiny():
        y = rg.Vector3d.YAxis
    y.Unitize()

    return rg.Plane(pt, x, y)


def _rect_profile_xy(
    plane: rg.Plane,
    depth: float,
    width: float,
) -> rg.Curve:
    """
    Rectangle in plane XY (depth along X, width along Y), centered.
    """
    return rg.Rectangle3d(
        plane,
        rg.Interval(-depth * 0.5, depth * 0.5),
        rg.Interval(-width * 0.5, width * 0.5),
    ).ToNurbsCurve()


def _post_brep_between_z(
    base_plane: rg.Plane,
    post_depth_mm: float,
    post_width_mm: float,
    z_bottom: float,
    z_top: float,
) -> rg.Brep:
    """
    Create a post that truly spans between two Z levels:
    z_bottom -> z_top (world Z).

    We do this by placing the profile at z_bottom and extruding up by (z_top - z_bottom).
    """
    h = float(z_top - z_bottom)
    if h <= 0:
        return None

    plane = rg.Plane(base_plane)
    plane.OriginZ += float(z_bottom)

    profile = _rect_profile_xy(plane, post_depth_mm, post_width_mm)
    ext = rg.Extrusion.Create(profile, h, True)
    return ext.ToBrep() if ext else None


def _rail_profile_YZ(
    plane: rg.Plane,
    depth: float,
    height: float,
) -> rg.Curve:
    """
    Rail profile explicitly in YZ plane (X is curve direction).
    """
    o = plane.Origin
    y = plane.YAxis
    z = plane.ZAxis

    p0 = o + y * (-depth * 0.5) + z * (-height * 0.5)
    p1 = o + y * (depth * 0.5) + z * (-height * 0.5)
    p2 = o + y * (depth * 0.5) + z * (height * 0.5)
    p3 = o + y * (-depth * 0.5) + z * (height * 0.5)

    return rg.Polyline([p0, p1, p2, p3, p0]).ToNurbsCurve()


def _sweep_rail(
    crv: rg.Curve,
    z: float,
    depth: float,
    height: float,
    lateral_offset: rg.Vector3d,
) -> List[rg.Brep]:
    """
    Sweep a rail aligned with posts.
    """
    rail_crv = crv.Duplicate()
    rail_crv.Transform(rg.Transform.Translation(lateral_offset))
    rail_crv.Transform(rg.Transform.Translation(0, 0, float(z)))

    plane = _frame_at(rail_crv, 0.0)
    profile = _rail_profile_YZ(plane, depth, height)

    sweep = rg.SweepOneRail()
    sweep.AngleToleranceRadians = 0.02
    sweep.ClosedSweep = False

    breps: List[rg.Brep] = []
    swept = sweep.PerformSweep(rail_crv, profile)
    if swept:
        for b in swept:
            b.CapPlanarHoles(0.01)
            breps.append(b)

    return breps


# ---------------------------------------------------------------------
# main
# ---------------------------------------------------------------------


def fence(
    guide,
    height_mm: float = 1200,
    post_spacing_mm: float = 2000,
    post_width_mm: float = 80,
    post_depth_mm: float = 80,
    bottom_rail: bool = True,
    top_rail: bool = True,
    middle_rail_count=1,
    rail_height_mm: float = 40,
    rail_depth_mm: float = 60,
    offset_mm: float = 0.0,
    embed_depth_mm: float = 0.0,
) -> List[rg.Brep]:
    """
    BIM-grade fence / railing.

    embed_depth_mm:
      Extends posts AND bottom rail downward below the guide curve (slab anchoring).

    IMPORTANT:
      Posts now *actually* start at -embed_depth_mm (moved down),
      not merely made taller.
    """

    crv = _coerce_curve(guide)

    # --------------------------------------------------
    # Sanitize inputs
    # --------------------------------------------------
    height_mm = float(height_mm)
    post_spacing_mm = float(post_spacing_mm)
    post_width_mm = float(post_width_mm)
    post_depth_mm = float(post_depth_mm)
    rail_height_mm = float(rail_height_mm)
    rail_depth_mm = float(rail_depth_mm)
    offset_mm = float(offset_mm)
    embed_depth_mm = max(0.0, float(embed_depth_mm))
    middle_rail_count = max(0, int(middle_rail_count))

    # --------------------------------------------------
    # Stable lateral offset
    # --------------------------------------------------
    start_plane = _frame_at(crv, 0.0)
    lateral_offset = start_plane.YAxis * offset_mm

    # --------------------------------------------------
    # Rail center Z positions
    # --------------------------------------------------
    rail_zs: List[float] = []

    if bottom_rail:
        # bottom rail is embedded with posts
        rail_zs.append(rail_height_mm * 0.5 - embed_depth_mm)

    for i in range(middle_rail_count):
        rail_zs.append((i + 1) * height_mm / (middle_rail_count + 1))

    top_rail_z = None
    if top_rail:
        top_rail_z = height_mm - rail_height_mm * 0.5
        rail_zs.append(top_rail_z)

    # --------------------------------------------------
    # Post Z extents (world Z relative to guide)
    # --------------------------------------------------
    z_bottom = -embed_depth_mm

    if top_rail_z is not None:
        z_top = top_rail_z + rail_height_mm * 0.5  # top of top rail
    else:
        z_top = height_mm  # fallback if no top rail

    # --------------------------------------------------
    # Posts
    # --------------------------------------------------
    length = crv.GetLength()
    post_count = max(2, int(round(length / post_spacing_mm)))
    params = [i / float(post_count - 1) for i in range(post_count)]

    breps: List[rg.Brep] = []

    for t in params:
        plane = _frame_at(crv, t)
        plane.Origin += lateral_offset

        post = _post_brep_between_z(
            plane,
            post_depth_mm,
            post_width_mm,
            z_bottom,
            z_top,
        )

        if post:
            breps.append(post)

    # --------------------------------------------------
    # Rails
    # --------------------------------------------------
    for z in rail_zs:
        breps.extend(
            _sweep_rail(
                crv,
                z,
                rail_depth_mm,
                rail_height_mm,
                lateral_offset,
            )
        )

    return breps
