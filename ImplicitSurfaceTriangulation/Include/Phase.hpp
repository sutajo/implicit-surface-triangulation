#pragma once
#include <optional>

template<class InputMesh, class OutputMesh = InputMesh>
class Phase
{
protected:
	InputMesh& inputMesh;
	OutputMesh& outputMesh;
public:
	Phase(InputMesh& inputMesh, OutputMesh& outputMesh) : inputMesh(inputMesh), outputMesh(outputMesh) {}
	virtual ~Phase() {}
	virtual void Start() {}
	virtual void RunIterations(int iterations) = 0;
	virtual void Run() { while (!Completed()) RunIterations(1); }
	virtual bool Completed() const = 0;
	InputMesh& GetInput() const { return inputMesh; }
	virtual OutputMesh& GetOutput() { return outputMesh; }
};

template<class InputMesh, class IntermediateMesh = InputMesh, class OutputMesh = InputMesh>
class PhaseChain : public Phase<InputMesh, OutputMesh>
{
protected:
	Phase<InputMesh, IntermediateMesh>& first_phase;
	Phase<IntermediateMesh, OutputMesh>& second_phase;
public:
	PhaseChain(
		Phase<InputMesh, IntermediateMesh> &first_phase, 
		Phase<IntermediateMesh, OutputMesh> &second_phase) 
		: Phase<InputMesh, OutputMesh>(first_phase.GetInput(), second_phase.GetOutput()), first_phase(first_phase), second_phase(second_phase)
	{
		first_phase.Start();
	}

	virtual ~PhaseChain() {}

	virtual void RunIterations(int iterations) override
	{
		if (!first_phase.Completed()) {
			first_phase.RunIterations(iterations);
			if (first_phase.Completed())
				second_phase.Start();
		}
		else
			second_phase.RunIterations(iterations);
	}

	virtual void Run() override
	{
		if (!first_phase.Completed())
			first_phase.Run();

		second_phase.Start();
		second_phase.Run();
	}

	bool HalfCompleted() const
	{
		return first_phase.Completed();
	}

	virtual bool Completed() const override
	{
		return second_phase.Completed();
	}
};