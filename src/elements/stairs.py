import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
from typing import List, Literal
import math

StairSide = Literal["left", "right", "center"]


# --------------------------------------------------
# Helpers
# --------------------------------------------------


def _coerce_polyline(curve) -> rg.Polyline:
    crv = rs.coercecurve(curve)
    if not crv:
        raise TypeError("guide must be a Curve")

    ok, pl = crv.TryGetPolyline()
    if not ok or pl.Count < 2:
        raise TypeError("guide must be a polyline")

    return pl


def _segment_direction(p0: rg.Point3d, p1: rg.Point3d) -> rg.Vector3d:
    v = p1 - p0
    v.Unitize()
    return v


def _left_vector(dir: rg.Vector3d) -> rg.Vector3d:
    left = rg.Vector3d.CrossProduct(rg.Vector3d.ZAxis, dir)
    left.Unitize()
    return left


def _alignment_offset(width: float, side: StairSide) -> float:
    if side == "center":
        return -width * 0.5
    if side == "left":
        return 0.0
    if side == "right":
        return -width
    raise ValueError("Invalid side")


# --------------------------------------------------
# Main stair
# --------------------------------------------------


def stair_from_polyline(
    guide,
    total_height_mm: float,
    riser_height_mm: float = 170,
    tread_depth_mm: float = 270,
    width_mm: float = 1200,
    side: StairSide = "center",
    tread_thickness_mm: float = 150,
    landing_depth_mm: float = 270,
) -> List[rg.Brep]:
    """
    BIM-style concrete stair with:
    - polyline reference axis
    - correct left / right / center alignment
    - explicit landings at kinks
    - global riser logic
    """

    pl = _coerce_polyline(guide)

    # --------------------------------------------------
    # Vertical logic
    # --------------------------------------------------
    riser_count = max(1, int(round(total_height_mm / riser_height_mm)))
    riser_height = total_height_mm / riser_count
    tread_count = riser_count - 1

    current_step = 0
    current_z = 0.0

    breps: List[rg.Brep] = []

    # --------------------------------------------------
    # Walk the polyline
    # --------------------------------------------------
    for i in range(pl.Count - 1):
        p0 = pl[i]
        p1 = pl[i + 1]

        dir = _segment_direction(p0, p1)
        left = _left_vector(dir)

        run_length = p0.DistanceTo(p1)
        steps_here = int(run_length // tread_depth_mm)

        # Base origin stays on reference line
        base_origin = p0 + left * _alignment_offset(width_mm, side)

        # ----------------------------------------------
        # Treads in this flight
        # ----------------------------------------------
        for s in range(steps_here):
            if current_step >= tread_count:
                break

            origin = (
                base_origin + dir * (s * tread_depth_mm) + rg.Vector3d.ZAxis * current_z
            )

            plane = rg.Plane(origin, dir, left)

            rect = rg.Rectangle3d(
                plane,
                rg.Interval(0, tread_depth_mm),
                rg.Interval(0, width_mm),
            ).ToNurbsCurve()

            ext = rg.Extrusion.Create(rect, tread_thickness_mm, True)
            if ext:
                breps.append(ext.ToBrep())

            current_step += 1
            current_z += riser_height

        # ----------------------------------------------
        # Landing at kink
        # ----------------------------------------------
        if i < pl.Count - 2 and current_step < tread_count:
            landing_origin = (
                base_origin
                + dir * (steps_here * tread_depth_mm)
                + rg.Vector3d.ZAxis * current_z
            )

            plane = rg.Plane(landing_origin, dir, left)

            rect = rg.Rectangle3d(
                plane,
                rg.Interval(0, landing_depth_mm),
                rg.Interval(0, width_mm),
            ).ToNurbsCurve()

            ext = rg.Extrusion.Create(rect, tread_thickness_mm, True)
            if ext:
                breps.append(ext.ToBrep())

    return breps
