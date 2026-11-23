#pragma once

enum class CartesianForceMode { force, hybrid, iac1 };

int runCartesianForceProgram(int argc, char* argv[], CartesianForceMode mode);
