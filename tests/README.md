# tests

KNOSSOS has no test harness, and CMake does not compile anything in this directory
(`CMakeLists.txt` globs `annotation/ mesh/ scriptengine/ segmentation/ slicer/ skeleton/
tinyply/ widgets/` only). These are standalone programs for the parts that are pure
computation and worth pinning down.

## distancetransform_test

Exercises `segmentation/distancetransform.h`, the signed Euclidean distance transform and
distance-field blend behind shape interpolation. Checks the sign convention, that
anisotropic voxel spacing is measured in nanometres rather than pixels, that interpolating
between two discs tracks the expected radius, that the key slices reproduce exactly at
t=0 and t=1, and that centroid alignment keeps a laterally drifting shape from pinching or
vanishing.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/dt_test distancetransform_test.cpp && /tmp/dt_test
```

Exits non-zero on failure.

## sislice_test

Exercises `segmentation/sislice.h`, the 2D key-slice mask. Mostly index arithmetic: global
coordinate ↔ mask index round-trips at magnification 1 and 4, that growing the bounding box
downward shifts the origin so painted voxels keep reading back at the same global
coordinate, that the origin stays on the magnification lattice, erase accounting, and
`shrinkToFit`.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/sislice_test sislice_test.cpp && /tmp/sislice_test
```

## viewportlayout_test

Exercises `widgets/viewportlayoutgeometry.h`, the placement arithmetic behind the named
viewport arrangements. Checks that the reference unit fits an arrangement into the window
(whichever of width and height binds), that nothing escapes the window at any aspect ratio,
that the built-in shapes are what they claim, and that capturing the current arrangement
and re-applying it reproduces it — including preserving empty space the user deliberately
left, which an earlier version stretched away.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/vplayout_test viewportlayout_test.cpp && /tmp/vplayout_test
```

## undohistory_test

Exercises `widgets/historytimeline.h`, the arithmetic behind the History window and the
undo budget: that a row in the history list maps to the right number of undo/redo steps
(with and without redoable states above the current one), that clicking a row lands on
that row, and that budget eviction respects both the entry-count and total-size caps while
never dropping the last remaining entry. This caught a sign error in the redo mapping that
would have jumped to the wrong state whenever more than one redo was available.

```bash
c++ -std=c++17 -O2 -I .. -o /tmp/undohistory_test undohistory_test.cpp && /tmp/undohistory_test
```
