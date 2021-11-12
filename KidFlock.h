//------------------------------------------------------------------------------
// File:    KidFlock.h
// Author:  Ryan Buehler
// Created: July, 2021
// Desc:    BOIDS flocking for children in Behavior Tree Demo (Custom Engine)
//          In the demo, there is a group of children that will flock toward
//          an adult in a BOIDS behavior flock around a beach. This class will
//          assist in keeping the flock operating cohesively.
//------------------------------------------------------------------------------
#pragma once

class KidFlock
{
public:
  /// <summary>
  /// Constructor
  /// </summary>
  KidFlock() noexcept;

  // Rule of 5
  ~KidFlock() = default;
  KidFlock(const KidFlock&) = delete;
  KidFlock& operator=(const KidFlock&) = delete;
  KidFlock(KidFlock&&) = delete;
  KidFlock& operator=(KidFlock&&) = delete;

  /// <summary>
  /// Adds a "kid" agent to the flock
  /// </summary>
  /// <param name="kid">[Ptr] </param>
  /// <returns></returns>
  void AddKid(BehaviorAgent* kid) noexcept;

  /// <summary>
  /// On Game Loop Tick, Update the "kid"
  /// </summary>
  /// <param name="kid">[Ptr] The kid agent to run the update iteration</param>
  /// <param name="vel">[Ref] The agent's velocity</param>
  /// <param name="dt">Frame Delta Time</param>
  void UpdateKid(BehaviorAgent* kid, Vec3& vel, float dt) noexcept;

  /// <summary>
  /// Sets the main target for this agent to follow
  /// </summary>
  /// <param name="target">[Ptr] The target agent to follow</param>
  /// <returns></returns>
  void SetTarget(BehaviorAgent* target) noexcept;

private:
  // Ideally, these would be updated via reflection, but that wasn't part of this project or engine
  static constexpr float SEP_POWER = 240.f; // Seperation power (BOIDS)
  static constexpr float ALI_POWER = 20.f;  // Aligment power (BOIDS)
  static constexpr float COH_POWER = 50.f;  // Cohesion power (BOIDS)
  static constexpr float TAR_POWER = 80.f;  // Target following power (BOIDS)

  static constexpr float SEP_THRESH = 18.f; // Seperation threshold - breakaway distance (BOIDS)

  static constexpr float MAX_SPEED = 11.f;  // Maximum agent physical speed

  std::vector<BehaviorAgent*> m_Kids; // Vector of [Ptr] to the "kid" agents
  BehaviorAgent* target_;             // A [Ptr] to the target to follow

  /// <summary>
  /// Helper function to apply separation part of BOIDS behavior
  /// </summary>
  /// <param name="kid">[Ptr] The kid agent</param>
  /// <param name="vel">[Ref] The kid's velocity</param>
  /// <param name="dt">Frame Delta Time</param>
  void ApplySeparation(BehaviorAgent* kid, Vec3& vel, float dt);

  /// <summary>
  /// Helper function to apply alignment part of BOIDS behavior
  /// </summary>
  /// <param name="kid">[Ptr] The kid agent</param>
  /// <param name="vel">[Ref] The kid's velocity</param>
  /// <param name="dt">Frame Delta Time</param>
  void ApplyAlignment(BehaviorAgent* kid, Vec3& vel, float dt);

  /// <summary>
  /// Helper function to apply cohesion part of BOIDS behavior
  /// </summary>
  /// <param name="kid">[Ptr] The kid agent</param>
  /// <param name="vel">[Ref] The kid's velocity</param>
  /// <param name="dt">Frame Delta Time</param>
  void ApplyCohesion(BehaviorAgent* kid, Vec3& vel, float dt);

  /// <summary>
  /// Helper function to apply the tendency to follow a target
  /// </summary>
  /// <param name="kid">[Ptr] The kid agent</param>
  /// <param name="vel">[Ref] The kid's velocity</param>
  /// <param name="dt">Frame Delta Time</param>
  void ApplyTarget(BehaviorAgent* kid, Vec3& vel, float dt);
};
