#pragma once

#define EXPORTAPI __declspec(dllexport) long __stdcall 


EXPORTAPI createOutputNodes(
	void * aHandle,
	void *aPtrUnkTarget,
	unsigned long aOutputNodeAmount,
	void** aRefOutputNodes);

EXPORTAPI getMaxOutputNodeCount(
	unsigned long* aPtrOutputNodeAmount);

EXPORTAPI createEVRStreamControl(
	void** aPtrPtrUnkIEVRStreamControl);