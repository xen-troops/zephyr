Trusted Execution Environment Driver Tests
##################

Those tests are designed to test the Generic TEE interface and OP-TEE driver
implementation. This are only Unit tests, that doesn't connect directly to
TrustZone OS and should check if driver api works as intended. The complete set
of the integration and unit tests is presented in [0], which is optee_test
adaptation for Zephyr.

Directories:
tee: The Generic tee interface implementation unit tests;
optee: Testing OP-TEE driver implementation API without real connection to
       the OP-TEE

[0] https://github.com/dsemenets/zephyr-optee-test.git
