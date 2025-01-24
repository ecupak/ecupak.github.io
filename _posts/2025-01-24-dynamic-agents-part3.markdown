---
layout: post
title:  "Dynamic Agents using Steering Behaviors and Flow Fields in C++ [3/3]"
date:   2025-01-24 13:55:45 +0100
categories: jekyll update
---

<h3 class="header1-swipe">Introduction</h3>

My recent student project at BUas (Breda University of Applied Science) covered steering behaviors and flow fields. 

The final product was a system that allowed agents to easily enable and disable the behaviors to use as well as adjust the settings of those behaviors. In addition, via ImGui you could adjust those same settings and see their effects during runtime - either on a per-agent basis or for all agents of a class.

I want to cover the ideas behind what I learned as well as share the code I implemented. I have divided this into 3 parts / posts. This is part 2 of 3:

- Part 1 covers the basics of steering behaviors, how they work with the physics system, and the C++ implementation of 9 steering behaviors (Seek, Flee, Pursue, Evade, Arrive, Wander, Separate, Align, and Gather).

- Part 2 covers the basics of flow fields, how to construct one, the C++ implementation of a working flow field, how to add bilinear interpolation, and how to handle dynamic objects moving across the flow field.

- **Part 3** covers how to combine the steering behavior and flow fields into a steering system, how to modify agents during runtime with new behaviors, and how the steering system and agents communicate.

I will explain everything within a 2D system / world. 

For my project, [EnTT](https://github.com/skypjack/entt) (an entity component system (ECS)) was utilized and was instrumental in putting all the pieces together; I have not considered the architecture for an engine without an ECS.

I also used [glm](https://github.com/g-truc/glm) as my math library. You'll see it in the C++ code snippets but should be able to easily swap it out for whatever you prefer.

<br/>

<h3 class="header1-swipe">Part 3: The Steering System</h3>

The goal of the steering system is to apply the steering behaviors to the agents and facilitate access to the flow field.

<h4 class="header2-swipe">The Components So Far</h4>

From part 1, we have:
- A Vehicle component
- A Personality component
- A Physics Body component
- A Layer component

Part 2 provided a new steering behavior and the underlying structure that supports it (a flow field), but no new components.

The component least explained so far has been the Personality component, so let's fix that.

<h4 class="header3-swipe">The Personality Component</h4>

As mentioned in Part 1, the Personality component holds all the behavior settings for an agent. In every steering behavior implementation, we saw that it was constantly referenced for data.

An additional responsibility of this component is to track which behaviors are active by using a bitfield.

By allowing an agent to modify their Personality component, they can change which behaviors are active and the settings. In my project, this was commonly done when an agent changed states:

```cpp
void WalkState::Enter(Personality& personality)
{	  
  // Flocking behavior settings.
  personality.m_separate.Set(
    true, // Exact (True) or overlap layer matching.
    1.0f, // Distance to look for neighbors.
    1.0f  // Weight.
  );
  personality.m_align.Set(true, 1.5f, 1.0f);
  personality.m_gather.Set(true, 2.0f, 0.2f);

  // Set layers to match.
  personality.m_separate.m_layer_list.SetFlag(Layer::SheepIdx);
  personality.m_align.m_layer_list.SetFlag(Layer::SheepIdx);
  personality.m_gather.m_layer_list.SetFlag(Layer::SheepIdx);
  
  // Behaviors.
  auto flocking_behavior{ sb::BehaviorFlag::fSeparate 
                        | sb::BehaviorFlag::fAlign 
                        | sb::BehaviorFlag::fGather };

  // Edit the bitfield of which behaviors are enabled.
  personality.m_behavior_list.SetFlag(flocking_behavior);  
}

And that was all the agent needed to do to change their behavior or alter a setting.
```

<h4 class="header2-swipe">The Steering System</h4>

The steering system has a pretty easy job. It iterates over all the agents with a Vehicle, Personality, and Physics Body component and checks which behaviors should be applied. An agent doesn't necessarily need a Layer component, but anything that the agent would like to check as a neighbor will need one. In practice, unless you have an one-off kind of agent, all your agents will have a Layer component.

The steering system also:
- Contains the flow field.
- Contains the steering behavior methods.
- Contains the kd-tree and updates it.

In addition, I made my steering system run before the physics system, because then the pipeline became:
1. Apply steering forces to velocity (steering system).
2. Use velocity to move the agent (physics system).
3. Get new input (gameplay).

.. and that made a lot of sense to me.

The actual code of my steering system processing the agents and applying the steering behaviors:

```cpp
void SteeringSystem::ApplySteeringForces(float)
{
  for (const auto& [entity, vehicle, personality, body] : ECS().Registry.view<Vehicle, Personality, const PhysicsBody>().each())
  {
      auto enabled_behaviors{ personality.m_behavior_flags };

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fSeek))
          SteeringBehaviors::Seek(vehicle, personality, body);
      
      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fFlee))
          SteeringBehaviors::Flee(entity, vehicle, personality, body);

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fWander))
          SteeringBehaviors::Wander(vehicle, personality, body);

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fArrive))
          SteeringBehaviors::Arrive(vehicle, personality, body);

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fPursue))
          SteeringBehaviors::Pursue(entity, vehicle, personality, body);

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fEvade))
          SteeringBehaviors::Evade(entity, vehicle, personality, body);
      
      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fSeparate))
          SteeringBehaviors::Separate(entity, vehicle, personality, body, m_kd_tree);

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fAlign))
          SteeringBehaviors::Align(entity, vehicle, personality, body, m_kd_tree);

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fGather))
          SteeringBehaviors::Gather(entity, vehicle, personality, body, m_kd_tree);        

      if (CheckBitFlagOverlap(enabled_behaviors, BehaviorFlag::fFlowFieldFollowing))
      {
          // Find flow field force.          
          auto flow_field_force{ m_flow_field.Lookup(body.GetPosition(), personality.m_flow_field_following.m_use_bilinear) };

          // Apply.
          SteeringBehaviors::FlowFieldFollowing(entity, vehicle, personality, body, flow_field_force);
      }
  }
}
```

The steering behaviors just need to be passed the required info to do their job.

The Flow Field Following behavior is the odd-one out, as it must first consult the flow field before passing that information to the steering behavior. After reworking the flocking behaviors to accept the kd-tree as an argument, making the Flow Field Following behavior accept the flow field as an argument and moving that logic into the behavior seems like a good idea.

<h3 class="header1-swipe">Part 3 Conclusion</h3>

This part was extremely short, but it didn't feel right dropping it in at the end of either of Part 1 or Part 2.

By utilizing the Personality component to store the behavior settings and allowing the steering behavior methods to read from this data, the steering system itself is a simple system.

<h3 class="header1-swipe">&nbsp;</h3>

<div style="text-align: center;">
  <img src="/assets/media/buas_logo.png" type="png" style="border: 1px white solid; max-width: 100%;">
</div>