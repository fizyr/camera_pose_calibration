Definition of calibration_tag frame for asymmetric circles calibration plate
============================================================================

The asymmetric cicles calibration node in this package assumes this definition of the calibration tag to be present as a tf transform. Make sure the calibration_tag conforms to this definition, or you might get unexpected results.

Imagine the square which circumscribes the calibration plate. Position the calibration plate on a table such that the two corner points of this square which have calibration dots on them are towards you. The lower left dot then represents the zero point of the calibration_tag frame.

- The X axis points to the right.
- The Y axis points forwards away from you.
- The Z-axis points upwards, out of the calibration plate.

