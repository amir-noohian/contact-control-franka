#pragma once

enum class CartesianForceMode { force, hybrid, iac1, iac1_tracking };

int runCartesianForceProgram(int argc, char* argv[], CartesianForceMode mode);
