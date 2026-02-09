import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
from typing import Dict, Iterable, Optional, Union


# ---------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------


def _coerce_curve(crv) -> rg.Curve:
    crv = rs.coercecurve(crv)
    if not crv:
        raise TypeError("boundary must be a Curve")
    return crv


def _coerce_curves(crvs: Optional[Union[rg.Curve, Iterable]]) -> Iterable[rg.Curve]:
    if not crvs:
        return []

    if isinstance(crvs, (list, tuple)):
        return [_coerce_curve(c) for c in crvs]

    return [_coerce_curve(crvs)]


def _planar_slab(
    curve: rg.Curve,
    z_base: float,
    thickness: float,
    voids: Iterable[rg.Curve],
) -> Optional[rg.Brep]:
    """
    Create a planar slab by extruding a curve downward,
    with optional void subtraction.
    """

    # Base slab
    crv = curve.Duplicate()
    crv.Transform(rg.Transform.Translation(0, 0, float(z_base)))

    slab_ext = rg.Extrusion.Create(
        crv,
        -float(thickness),  # extrude DOWN
        True,
    )

    if not slab_ext:
        return None

    slab = slab_ext.ToBrep()

    # ---------------------------------------------
    # Void subtraction
    # ---------------------------------------------
    tol = 0.01

    for void in voids:
        void_crv = void.Duplicate()
        void_crv.Transform(rg.Transform.Translation(0, 0, float(z_base)))

        void_ext = rg.Extrusion.Create(
            void_crv,
            -float(thickness),
            True,
        )

        if not void_ext:
            continue

        void_brep = void_ext.ToBrep()

        result = rg.Brep.CreateBooleanDifference(
            slab,
            void_brep,
            tol,
        )

        if result and len(result) > 0:
            slab = result[0]

    return slab


# ---------------------------------------------------------------------
# main
# ---------------------------------------------------------------------


def floor_plate(
    boundary,
    elevation_mm: float,
    finish_thickness_mm: float = 15,
    screed_thickness_mm: float = 70,
    insulation_thickness_mm: float = 30,
    structural_thickness_mm: float = 250,
    voids=None,
) -> Dict[str, rg.Brep]:
    """
    Multi-layer floor build-up (top → bottom).

    Optional void curves are subtracted from all layers
    (e.g. stair openings, shafts).

    Returns:
      {
        "finish": Brep,
        "screed": Brep,
        "insulation": Brep,
        "structural": Brep
      }
    """

    boundary = _coerce_curve(boundary)
    voids = _coerce_curves(voids)

    z = float(elevation_mm)
    layers: Dict[str, rg.Brep] = {}

    layers["finish"] = _planar_slab(boundary, z, finish_thickness_mm, voids)
    z -= finish_thickness_mm

    layers["screed"] = _planar_slab(boundary, z, screed_thickness_mm, voids)
    z -= screed_thickness_mm

    layers["insulation"] = _planar_slab(boundary, z, insulation_thickness_mm, voids)
    z -= insulation_thickness_mm

    layers["structural"] = _planar_slab(boundary, z, structural_thickness_mm, voids)

    return layers
