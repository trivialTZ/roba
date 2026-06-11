# Pork-Belly Three-Layer Material Model (Aim 1.2)

Parameter priors for the skin / fat / lean-muscle layers, with provenance and honest uncertainty.
**These are order-of-magnitude priors for domain randomization, not validated constants** — cutting
physics depends on fracture mechanics (energy/area), blade sharpness, and friction, and the literature
carries order-of-magnitude scatter, juvenile-vs-adult mismatch, and (for muscle) severe anisotropy.
Plan to **calibrate locally** against benchtop cutting/force tests. Sources in `REFERENCES.md` (Pillar 4).

## Layer parameters

| Property | **Skin (dermis/rind)** | **Subcutaneous fat** | **Lean muscle** | Notes |
|----------|------------------------|----------------------|-----------------|-------|
| Fracture toughness `Jc` | **17–30 kJ/m²** | **~4.1 kJ/m²** (measured) | **0.1–0.84 kJ/m²** (±~100%) | Skin dominates initial-cut resistance; ~4× fat |
| Elastic stiffness | MPa-scale (tension), hyperelastic | shear modulus ~1–5 kPa | tens–hundreds kPa, **anisotropic** | single scalar `E` not meaningful for skin/muscle |
| Anisotropy | aligned collagen | ~isotropic | **strong** (fiber direction) | muscle: model fiber axis explicitly |
| Density | — | — | — | bulk pork ~1050 kg/m³ (with bone, raw) |
| Blade-tissue friction μ | **data gap** — treat as tunable (~0.1–0.5, lower with rendered-fat self-lubrication); ~30–45% of cut force | sensitivity-sweep this | |

### Provenance & caveats
- **Skin:** best data is *juvenile piglet* skin (Comley & Fleck 2010 ≈17 kJ/m²; Pissarenko 2020 ≈20–30 kJ/m²);
  order-of-magnitude revisions across studies; hyperelastic, so no single `E`.
- **Fat:** Comley & Fleck 2010 trouser-tear `Jc ≈ 4.1 kJ/m²` (porcine) — a real measured value; the early
  "no fat data" worry was refuted. Compressive modulus varies 30–60× by depot/condition.
- **Muscle:** Taylor 2012 judged most soft-tissue toughness measurements invalid; the one porcine-muscle
  `Jc` paper reports ~100% relative scatter; highly fiber-direction & temperature dependent.
- **Force / cutting:** Warner-Bratzler shear and robotic-deboning forces are *process/tenderness* metrics
  tied to specific blade geometry & cook state — **not** transferable material constants.

## Cutting-force model (for DiSECt / breakable-seam thresholds)

Use an **Atkins/Lucas energy-based** form:

```
work_to_cut ≈ Jc · (cut area)  +  friction term  +  plastic/deformation term
```

- **Slice-push (pressing-slicing) ratio** lowers required normal force by **~40%** vs pure pressing —
  expose it as a control variable, not a constant.
- **Breakable-seam mapping (interactive demo):** per-layer joint break-force ∝ `Jc · seam_cross_section`,
  preserving the ordering **skin ≫ fat > lean**; modulate by slice-push ratio.
- **DiSECt (physics path):** set the per-layer cohesive/failure-energy parameter to `Jc`; calibrate the
  remaining params (friction, damping) to real force-vs-depth data via DiSECt's differentiable param ID.

## Recommended practice
1. Initialize from the table above; 2. **domain-randomize** over the listed ranges; 3. collect a small
real cutting dataset (trouser/wedge tests + instrumented blade force-vs-depth) and **calibrate** —
especially the friction coefficient and the fiber-direction-dependent muscle response, which the
literature does not supply ready-made.
