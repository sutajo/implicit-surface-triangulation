#pragma once

#include "ImplicitObject.hpp"
#include "GrowingPhase.hpp"
#include "FillingPhase.hpp"

namespace Implicit
{
	namespace Tessellation
	{
		struct KarkanisStewartState
		{
			GlmPolyMesh mesh;
		};

		struct KarkanisStewartSubPhases
		{
			GrowingPhase growingPhase;
			FillingPhase fillingPhase;

			KarkanisStewartSubPhases(GlmPolyMesh &mesh, Object& object): 
				growingPhase(mesh, object),
				fillingPhase(mesh, object) 
			{}
		};

		/**
		 Generates a triangle mesh for a given implicit object
		 */
		class KarkanisStewartTessellator : public KarkanisStewartState, public KarkanisStewartSubPhases, public PhaseChain<GlmPolyMesh>
		{
		public:
			KarkanisStewartTessellator(Object& object);
		};
	}
};