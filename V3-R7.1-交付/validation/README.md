# R7.1 validation note

`R7.1_existing_R7_data_bias_sanity.csv` is computed only from the existing R7 diagnostic residual columns. It validates the expected sign and scale of a common rope-length bias but does not replace a compiled R7.1 synthetic test or a new 82-pose field acquisition.

The modified C++ math core was also compiled natively with g++ + Eigen in this environment. See `native_cpp/V3SyntheticTest_R7.1_linux.txt` and `native_cpp/V3GradientCheck_R7.1_linux.txt`. The compiled Linux executables themselves are intentionally not included in the delivery.
