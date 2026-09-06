#pragma once

void acceptance_manifest_record(const char* name, bool passed);

void run_acceptance_iris();
void run_acceptance_arq();
void run_acceptance_native();
void run_acceptance_rc5surg();

void run_acceptance_gate(int& passed, int& failed);
