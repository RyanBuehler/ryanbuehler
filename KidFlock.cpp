//------------------------------------------------------------------------------
// File:    KidFlock.cpp
// Author:  Ryan Buehler
// Created: July, 2021
// Desc:    BOIDS flocking for children in Behavior Tree Demo (Custom Engine)
//          In the demo, there is a group of children that will flock toward
//          an adult in a BOIDS behavior flock around a beach. This class will
//          assist in keeping the flock operating cohesively.
//------------------------------------------------------------------------------
#include "pch.h"
#include "KidFlock.h"
#include <math.h>

KidFlock::KidFlock() noexcept :
	m_Kids(), target_(nullptr)
{}

void KidFlock::AddKid(BehaviorAgent* kid) noexcept
{
	// Null reference check
	if (kid == nullptr)
		return;

	// Check if the kid is already tracked in the vector
	const auto& it = std::find(m_Kids.begin(), m_Kids.end(), kid);
	if(it == m_Kids.end())
		m_Kids.push_back(kid);
}

void KidFlock::UpdateKid(BehaviorAgent* kid, Vec3& vel, float dt) noexcept
{
	// Null reference check
	if (kid == nullptr)
		return;

	// No updates if there are no agents
	if (m_Kids.empty())
		return;

	// Apply respective BOIDS independent behaviors
	ApplySeparation(kid, vel, dt);
	ApplyAlignment(kid, vel, dt);
	ApplyCohesion(kid, vel, dt);

	// If a target exists, move toward target
	if (target_ != nullptr)
	{
		ApplyTarget(kid, vel, dt);
	}

	// Limit speed
	if (vel.Length() > MAX_SPEED)
	{
		vel *= MAX_SPEED / vel.Length();
	}

	// Interface with engine to turn the agent appropriately
	float angle = atan2f(vel.x, vel.z);
	kid->set_yaw(angle);
}

void KidFlock::SetTarget(BehaviorAgent* target) noexcept
{
	// Null reference check
	if (target == nullptr)
		return;

	target_ = target;
}

void KidFlock::ApplySeparation(BehaviorAgent* kid, Vec3& vel, float dt)
{
	// Null reference check
	if (kid == nullptr)
		return;

	// Iterate through all kids
	for (BehaviorAgent* other : m_Kids)
	{
		// Don't apply if this is the same kid
		if (other == kid)
			continue;

		const Vec3& opos = other->get_position();
		const Vec3& kpos = kid->get_position();

		// Specific threshold, therefore distance squared may be used
		float dist = Vec3::DistanceSquared(opos, kpos);

		// If too close to another kid, apply separation
		if (dist < SEP_THRESH)
		{
			float factor = 1.f - dist / SEP_THRESH;
			factor = std::clamp(factor, 0.5f, 1.f);
			Vec3 away = kpos - opos;
			away.y = 0.f;
			away.Normalize();
			away *= SEP_POWER;
			vel += away * dt;
		}
	}
}

void KidFlock::ApplyAlignment(BehaviorAgent* kid, Vec3& vel, float dt)
{
	// Null reference check
	if (kid == nullptr)
		return;

	Vec3 avgVel = Vec3(); // For collecting all velocities and averaging

	// Iterate through all kids
	for (BehaviorAgent* other : m_Kids)
	{
		// Don't apply if this is the same kid
		if (other == kid)
			continue;

		// Get the other kid's velocity
		auto& bb = other->get_blackboard();
		Vec3 ovel = bb.get_value<Vec3>("Velocity");
		avgVel += ovel;
	}

	// Average the velocities
	avgVel /= static_cast<float>(m_Kids.size());
	// Normalize to get the pure direction vector
	avgVel.Normalize();

	// Apply alignment
	vel += avgVel * ALI_POWER * dt;
}

void KidFlock::ApplyCohesion(BehaviorAgent* kid, Vec3& vel, float dt)
{
	// Null reference check
	if (kid == nullptr)
		return;

	Vec3 avgPos = Vec3(); // For collecting all positions and averaging

	// Iterate through all kids
	for (BehaviorAgent* other : m_Kids)
	{
		// Don't apply if this is the same kid
		if (other == kid)
			continue;

		// Get the other kid's position
		Vec3 opos = other->get_position();
		avgPos += opos;
	}

	// Average the positions
	avgPos /= static_cast<float>(m_Kids.size());

	// Find the new direction for the agent to move
	Vec3 newDir = avgPos - kid->get_position();
	// Normalize to get the pure direction vector
	newDir.Normalize();

	// Apply Cohesion
	vel += newDir * COH_POWER * dt;
}

void KidFlock::ApplyTarget(BehaviorAgent* kid, Vec3& vel, float dt)
{
	// Null reference check
	if (kid == nullptr)
		return;

	// Head in the direction of the target
	Vec3 newDir = target_->get_position() - kid->get_position();

	// Normalize for pure direction
	newDir.Normalize();

	// Apply Target direction
	vel += newDir * TAR_POWER * dt;
}
