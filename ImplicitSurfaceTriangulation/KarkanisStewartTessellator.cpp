#include "KarkanisStewartTessellator.hpp"

Implicit::Tessellation::KarkanisStewartTessellator::KarkanisStewartTessellator(Object& object): 
	KarkanisStewartSubPhases(mesh, object),
	PhaseChain(growingPhase, fillingPhase)
{
	omerr().rdbuf(std::cerr.rdbuf());
}