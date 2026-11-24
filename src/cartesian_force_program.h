#pragma once

enum class CartesianForceMode { force, hybrid, iac1, iac1_tracking, iac1_hybrid };

int runCartesianForceProgram(int argc, char* argv[], CartesianForceMode mode,
                             const char* program_name = nullptr);
