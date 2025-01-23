---
layout: post
title:  "Dynamic Agents using Steering Behaviors and Flow Fields in C++"
date:   2025-01-22 13:55:45 +0100
categories: jekyll update
---

<h3 class="header1-swipe">Introduction</h3>

My recent student project at BUas (Breda University of Applied Science) covered steering behaviors and flow fields. 

The final product was a system that allowed agents to easily enable and disable the behaviors to use as well as adjust the settings of those behaviors. In addition, via ImGui you could adjust those same settings and see their effects during runtime - either on a per-agent basis or for all agents of a class.

In this post I want to cover the ideas behind what I learned as well as share the code I implemented. I have divided this post into 3 parts:

- **Part 1** covers the basics of steering behaviors, how they work with the physics system, and the C++ implementation of 9 steering behaviors (seek, flee, pursue, evade, arrive, wander, separate, align, and gather).

- **Part 2** covers the basics of flow fields, how to construct one, the C++ implementation of a working flow field, how to add bilinear interpolation, and how to handle dynamic objects moving across the flow field.

- **Part 3** covers how to combine the steering behavior and flow fields into a steering system, how to modify agents during runtime with new behaviors, and how the steering system and agents communicate.

I will explain everything within a 2D system / world. 

For my project, [EnTT](https://github.com/skypjack/entt) (an entity component system (ECS)) was utilized and was instrumental in putting all the pieces together; I have not considered the architecture for an engine without an ECS. The value of Part 3 may vary wildly depending on if you are using an ECS or not.

I also used [glm](https://github.com/g-truc/glm) as my math library. You'll see it in the C++ code snippets but should be able to easily swap it out for whatever you prefer.

<br/>

<h3 class="header1-swipe">Part 1: Steering Behaviors</h3>

A vast majority of my knowledge about steering behaviors comes from the [paper published by Craig Reynolds](https://www.red3d.com/cwr/steer/gdc99/). I encourage you to find time to read through it if you add steering behaviors to your game.

To those that are already familiar with the behaviors he describes, be aware that I slightly renamed some of them in my project. There's no big justification - they simply fit the way I talked about my code.

<h4 class="header2-swipe">How They Work</h4>

Every steering behavior works by calculating a steering force and applying it to the agent's velocity. The steering force is the difference between the agent's velocity and the desired velocity of the behavior (thus it steers the agent towards where the behavior wants them to go).

Calculating the steering force will be covered later in the C++ implementation section. Let's first look at what we do once we have the steering force:
- The steering force is truncated by the agent's maximum allowed force.
- Acceleration is calculated and applied to the agent's velocity.
- The agent's velocity is then truncated by the agent's maximum allowed velocity.
- Finally, the agent's velocity is applied to their position.

You can easily imagine that the agent's maximum force and velocity might change depending on how they are moving around the world - running on foot, in a car, or on a horse. All these different methods of moving around are contained in a Vehicle class (don't get too wrapped up in the name Vehicle - it's just a name to symbolize a method of moving).

The Vehicle class holds information about the method of movement: max force, max speed, orientation (the forward vector), and any additional information that is related to movement (such as a turning radius). Using the bullet list above, my Vehicle class has the following method:

```cpp
void Vehicle::FinalizeSteeringForce(float dt, PhysicsBody& body)
{
  // Truncate by max force.
  m_steering_force = Truncate(m_steering_force, m_max_force);
  
  // Apply acceleration.
  auto acceleration{ m_steering_force * body.GetInvMass() * dt };
  
  // Get truncated desired velocity.
  auto desired_velocity{ Truncate(m_velocity + acceleration, m_max_speed) };
  
  // ... Apply it.
  body.SetVelocity(body.GetVelocity() + (desired_velocity - m_velocity));
  
  // Store current velocity and position.
  m_velocity = body.GetVelocity();
  m_position = body.GetPosition();
}
```

This all lines up with what I previously said except for the line:

```cpp
body.SetVelocity(body.GetVelocity() + (desired_velocity - m_velocity));
```

The reason I apply the delta velocity of the vehicle instead of simply applying the full velocity is that I want my agents to still react to the world around them.

Consider a scenario where your agent has been launched from a cannon: if the agent's velocity is set based on the Vehicle's max velocity, your agent would suddenly lose all the velocity caused by the cannon.

The final lines store the velocity and position of the physics body in the vehicle for future use. `m_velocity` is used within this method to help get the delta velocity. `m_position` is used elsewhere in situations where accessing the physics body would have been inconvenient.

An imporant Vehicle method that will appear later is `AddSteeringForce()`. It's simple, so let's look at it now:

```cpp
void Vehicle::AddSteeringForce(const glm::vec2& force)
{
  m_steering_force += force;
}
```

And to finish out this section, I'll show what `Truncate()` does:

```cpp
glm::vec2 Vehicle::Truncate(const glm::vec2& current_v, const float max_v) const
{
  // Avoiding square roots...
  auto max2{ max_v * max_v};
  auto current2{ glm::length2(current_v) };
  
  // Vector is within limits.
  if (current2 <= max2) return current_v;
  
  // Reduce vector to maximum.
  return glm::normalize(current_v) * max_v;
}
```

<h4 class="header2-swipe">The Physics System</h4>

We are not going to create a physics system in this post. In fact, the steering system does not need to talk to a physics system directly.

But the steering system does assume the agent has a physics body with velocity, as the Vehicle applies the steering force directly to the agent's velocity (as seen earlier).

My project used a physics system that applied semi-implicit Euler integration using forces and velocity to obtain positions (here's [a nice blogpost covering integration methods](https://www.gorillasun.de/blog/euler-and-verlet-integration-for-particle-physics/) if that interests you).

If you are hoping to recreate what I've done, you'll need a way to calculate position that looks like this:

```cpp
void PhysicsBody::Update(float dt)
{
  m_velocity += m_force * m_invMass * dt;
  m_position += m_velocity * dt;
}
```

`m_force` will never contain a steering force (those are handled by the Vehicle and applied to velocity). This variable would be the total forces caused by other things, like impulses from collisions or wind from the environment.

<h4 class="header2-swipe">An Agent's Personality</h4>

Every steering behavior has parameters that can be adjusted. We don't want to hardcode these values into the steering behavior, so instead I created a Personality struct that holds all of these parameters. Each agent has their own Personality.

The Personality struct will be discussed more in Part 3, but I wanted to introduce it now because it shows up when discussing the steering behaviors.

<h4 class="header2-swipe">Layers</h4>

The final thing to discuss before looking at the steering behaviors is a small component called Layers, which is a bitfield. Each agent can belong to one or more layer, each represented as a bitflag.

This allows an agent to quickly identify other agents and if they are of interest. Each behavior, if it needs one, has its own layer bitfield that can be compared to other agents' layer.

<h4 class="header2-swipe">Steering Behaviors</h4>

Let's look at the steering behaviors that I was able to add to my project. At the core, these are all *heavily* based on the description given in Craig Reynolds' paper. My additions are mainly to catch errors and avoid any issues that can arise from an invalid Personality setting (for example, trying to use the Seek behavior while not having any target to actually go towards).

Each of these behaviors are a static method because they exist in the steering system - part of the game's engine - and only modify the steering force of the Vehicle and sometimes store a value in the Personality struct. 

For each behavior, I'll provide a brief explanation of what it does, the code in C++, any additional explanation of the code, and then a gif of what this behavior looks like in action in my project.

In the code, you'll notice that anytime I normalize a vector I also check the result for `nan` values to make sure it didn't have a length of 0; I discard any of those results. I could have also checked the length of the vector before normalizing to see if it was 0, and perhaps that would have been a better approach as it skips the cost of normalizing if the result will not be used.

The gif will show the autonomous agent in white and the player-controlled character in green. The agent has a yellow line connected to it that represents the current velocity. Connected to the end of the velocity line is a blue line that represents the steering force. Other debugging visuals are described as they appear. In general, the colors are poorly chosen, but opening the gif in a new tab will enlarge it and improve the quality.

Remember that we have a Vehicle class, a Personality struct, and a PhysicsBody that will be used by each behavior.

<h4 class="header3-swipe">Seek</h4>

The Seek behavior is the simplest: it moves your agent towards a target location. The agent will swing back and forth over the target location once it has been reached.

```cpp
static void Seek(Vehicle& vehicle, Personality& data, const PhysicsBody& body)
{
  if (!data.m_has_target)
  
  auto desired_velocity{ glm::normalize(data.m_target_position - body.GetPosition()) * vehicle.GetMaxSpeed() };

  // Confirm current position is not already at target.
  // Both components will be nan if normalizing a 0-vector.
  if (!std::isnan(desired_velocity.x))
  {
    auto force{ desired_velocity - body.GetVelocity() };
    vehicle.AddSteeringForce(data.m_seek.m_weight * force);
  }  
}
```

In this behavior and in many others, the desired velocity is the normalized vector between the agent's velocity and the position the behavior wants the agent to be at, multiplied by the agent's max speed. This is because it is assumed the agent would move at their max speed if possible. This was a parameter that I had been wanting to play with but never got around to doing.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/seek.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Seek behavior in action. The white square is the agent's target.</em>
    </figcaption>
</figure>

<h4 class="header3-swipe">Flee</h4>

This is very similar to Seek, but the agent will move away from the target instead.

```cpp
static void Flee(Vehicle& vehicle, Personality& data, const PhysicsBody& body)
{
  glm::vec2 total_flee_force{ 0.0f, 0.0f };

  float safe_distance{ data.m_flee.m_distance * data.m_flee.m_distance };

  // Check all entities to find the ones considered threats.
  for (const auto& [entity, layer, other_body] : ECS().Registry.view<Layer, PhysicsBody>().each())
  {
    if (entity == me) continue;

    // Check if entity is something to flee from.
    if (CheckBitFlagOverlap(data.m_flee.m_layer_list, layer.Idx))
    {
      // Only flee if entity is too close.
      if (float distance_to_threat{ glm::distance2(body.GetPosition(), other_body.GetPosition()) };
        distance_to_threat < safe_distance)
      {
        auto desired_velocity{ glm::normalize(body.GetPosition() - other_body.GetPosition()) * vehicle.GetMaxSpeed() };

        if (!std::isnan(desired_velocity.x))
        {
          total_flee_force += desired_velocity - body.GetVelocity();
        }
      }
    }
  }

  vehicle.AddSteeringForce(data.m_flee.m_weight * total_flee_force);
}
```

The big difference with Flee is that it can take into account multiple different agents and calculate a steering force that gets the agent away from all of them.

There's a line in there using the ECS. It basically loops over every entity that has both a Layer and PhysicsBody. Astute readers will realize this is likely going to be a problem when the number of agents creeps too high. We'll see a solution I implemented later on in other behaviors to combat this; I simply never got around to adding it to all of the behaviors.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/flee.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Flee behavior in action. The agent flees from the player. The purple circle around the agent is the distance it looks for any agent to potentially flee from.</em>
    </figcaption>
</figure>

<h4 class="header3-swipe">Pursue</h4>

In Pursue, the agent predicts where the target will be and moves towards that position. This of course only works if the target is something that can move, otherwise it is an expensive Seek.

But, like Seek, Pursue can only have a single target. If you tried to give an agent multiple targets to chase after, it would chase the average of their positions and fail to chase any single target effectively. That can't happen in my implementation, as the Personality struct only holds a single target for the Pursue behavior.

```cpp
static void Pursue(Vehicle& vehicle, Personality& data, const PhysicsBody& body)
{
  if (!data.m_has_target) return;

  // If target has a body, predict its movement.
  if (auto* target_body{ ECS().Registry.try_get<PhysicsBody>(data.m_target) })
  {
    // Get prediction.
    float my_speed{ glm::length(body.GetVelocity()) };
    auto predicted_point{ GetPredictedPoint(body, my_speed, *target_body) };

    // If offset distance is non-zero, apply offset to predicted point.
    if (data.m_pursue.m_offset_distance > 0.0f)
    {
      if (auto* target_vehicle{ ECS().Registry.try_get<Vehicle>(data.m_target) })
      {
        // Add target's current orientation angle.
        const auto& target_heading{ target_vehicle->GetOrientation() };
        float target_angle{ atan2f(target_heading.y, target_heading.x) };

        // Offset the point by combining target's angle with offset angle.
        predicted_point.x += cosf(data.m_pursue.m_offset_angle + target_angle) * data.m_pursue.m_offset_distance;
        predicted_point.y += sinf(data.m_pursue.m_offset_angle + target_angle) * data.m_pursue.m_offset_distance;
      }
    }

    // Seek targt.
    auto desired_velocity{ glm::normalize(predicted_point - body.GetPosition()) * vehicle.GetMaxSpeed() };

    if (!std::isnan(desired_velocity.x))
    {
      auto force{ desired_velocity - body.GetVelocity() };
      vehicle.AddSteeringForce(data.m_pursue.m_weight * force);
    }
  }
}
```

There are additional ECS lines here. Both receive a pointer to the component if it exists; otherwise it is a `nullptr`.

A shared `GetPredicatedPoint()` method is used by both Pursue and Evade (the next behavior). It looks like this:

```cpp
static glm::vec2 GetPredictedPoint(const PhysicsBody& body, float my_speed, const PhysicsBody& target_body)
{
  // Predict ahead more when further away.
  float distance_from_target{ glm::length(body.GetPosition() - target_body.GetPosition()) };

  // Predict ahead more when speeds are greater. If would divide by zero, set to 1.
  float total_speed{ my_speed + glm::length(target_body.GetVelocity()) };
  float inverse_speed{ total_speed > 0.0f ? 1.0f / total_speed : 1.0f };

  // Final prediction.
  float look_ahead_time{ distance_from_target * inverse_speed };
  return target_body.GetPosition() + target_body.GetVelocity() * look_ahead_time;
}
```

This logic was taken from Mat Buckland's book, *Programming Game AI by Example (2005)*, and works very well.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/pursue.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Pursue behavior in action. The blue square is the predicted point that the agent is steering towards.</em>
    </figcaption>
</figure>

<h4 class="header3-swipe">Evade</h4>

Evade is, at its heart, the same as Pursue, but the agent moves away from the target. However, like Flee, Evade is capable of considering multiple agents to avoid.

Because Evade is more active in the way it avoids other agents, I allowed it to specify specific agents to evade. For example, an animal doesn't run away from all humans, but will evade the ones that have hurt it in the past.

This added ability to Evade agents in a layer and specific agents does make the method far larger than anything else so far, but breaking down the steps should make it more digestable:
1. Find all agents that match the Evade layer, and, if within range, predict their future locations.
2. Find all specific agents in the Evade list, and, if within range, predict their future locations.
3. Sum up all of the steering forces that move the agent away from each predicted location.
4. Apply the summed steering forces. 

```cpp
static void Evade(Entity me, Vehicle& vehicle, Personality& data, const PhysicsBody& body)
{
  // Get future position of target(s).
  std::vector<glm::vec2> predicted_points;

  // Precalculated values.
  float my_speed{ glm::length(body.GetVelocity()) };
  float safe_distance{ data.m_evade.m_distance * data.m_evade.m_distance };

  // Check all targets in target list.
  for (const auto& e : data.m_evade.m_target_list)
  {
    if (e == me) continue;

    if (auto* target_body{ ECS().Registry.try_get<const PhysicsBody>(e) })
    {
      // Only flee if entity is too close.
      if (float distance_to_threat{ glm::distance2(body.GetPosition(), target_body->GetPosition()) };
        distance_to_threat < safe_distance)
      {
        predicted_points.push_back(GetPredictedPoint(body, my_speed, *target_body));
      }
    }
  }

  // Check all targets that match the layer list.
  if (!data.m_evade.m_layer_list.IsEmpty())
  {
    for (const auto& [entity, layer, target_body] : ECS().Registry.view<Layer, const PhysicsBody>().each())
    {
      if (entity == me) continue;

      // Check if entity is something to flee from.
      if (CheckBitFlagOverlap(data.m_evade.m_layer_list.Get(), layer.Idx))
      {
        // Only flee if entity is too close.
        if (float distance_to_threat{ glm::distance2(body.GetPosition(), target_body.GetPosition()) };
          distance_to_threat < safe_distance)
        {
          predicted_points.push_back(GetPredictedPoint(body, my_speed, target_body));
        }
      }
    }
  }

  // Flee from target(s).
  glm::vec2 total_evade_force{ 0.0f, 0.0f };

  for (const auto& predicted_point : predicted_points)
  {
    auto desired_velocity{ glm::normalize(body.GetPosition() - predicted_point) * vehicle.GetMaxSpeed() };

    if (!std::isnan(desired_velocity.x))
    {
      total_evade_force += desired_velocity - body.GetVelocity();				
    }
  }

  vehicle.AddSteeringForce(data.m_evade.m_weight * total_evade_force);
}
```

There shouldn't be anything new in the above code that hasn't appeared in Flee or Pursue.

Like Flee, the loop that checks every agent for any that are in range will suffer when the agent count is high. You'll see an improved method for this soon, and I honestly should add that solution into this behavior as well.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/evade.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Evade behavior in action. The agent flees from the player. The purple circle around the agent is the distance it looks for any agent to potentially flee from. The yellow square is the predicted point that the agent is steering away from.</em>
    </figcaption>
</figure>

<h4 class="header3-swipe">Wander</h4>

Wander is one of my favorite behaviors. It lets the agent meander about the world, making their own path forward. This is done by jittering a target location ahead of the agent.

To encourage the agent to carve out a smooth path, the jittering is constrained by using 2 circles. My names for the circles and their adjustable radii is honestly a bit messy, so bear with me:
- The larger circle is the *future circle* and its radius is called its *strength*.
- The smaller circle is the *target circle* and its radius is called its *rate*.

I know the code below would be hard to visualize for me if I was reading it for the first time, so here's a visualized summary:

<div style="text-align: center;">
  <img src="/assets/media/wander_help.png" type="png" style="border: 1px white solid; max-width: 100%;">
</div>
<br/>
1. The center of the future circle (blue) is calculated based on the position, orientation, and max speed of the agent (triangle). The previously used target location is found (black dot) and will be the center of the target circle.
2. A random point (orange) is placed on the target circle (black). The point is the rough target location.
3. The vector (white) from the center of the future circle to the rough target location is calculated. The actual target location (black) is then mapped to the circumference of the future circle.

The previous target location is used as input to calculate the next target location. If this is the first frame of Wander being active, you could randomize the value (a unit circle vector multiplied by the radius of the future circle) or reuse the last calculated value if it's not that important.

By jittering the target location like this, the frame-to-frame change in desired velocity never moves too much. By adusting the radius of both circles you can achieve a looser or tighter wandering path.

```cpp
static void Wander(Vehicle& vehicle, Personality& data, const PhysicsBody& body)
{
  // Find future point of the agent.
  glm::vec2 future_center{ body.GetPosition() + (vehicle.GetOrientation() * vehicle.GetMaxSpeed()) };

  // Find center of target circle. Use target data from previous frame calculation.		
  auto target_center{ future_center + data.m_wander.m_target_offset };

  // Find random point on target circle perimeter (the rough wander target).  
  float target_angle{ GetRandomNumber(0.0f, glm::two_pi<float>()) };
  auto rough_wander_target{ target_center + glm::vec2{ data.m_wander.m_rate * cosf(target_angle), data.m_wander.m_rate * sinf(target_angle) } };

  // Place rough wander target on circumference of future circle.
  auto center_to_target{ rough_wander_target - future_center };
  data.m_wander.m_target_offset = glm::normalize(center_to_target) * data.m_wander.m_strength;

  // Set new target.
  data.m_has_target = true;
  data.m_target_position = future_center + data.m_wander.m_target_offset;

  // Seek to target.
  auto desired_velocity{ glm::normalize(data.m_target_position - body.GetPosition()) * vehicle.GetMaxSpeed() };

  if (!std::isnan(desired_velocity.x))
  {
    auto force{ desired_velocity - body.GetVelocity() };
    vehicle.AddSteeringForce(data.m_wander.m_weight * force);
  }
}
```

The actual value saved is the offset from the future circle's center and not the actual target location. This is because we want this position to be relative to the future circle.

If we used the actual previous target location, and the agent was moving 10 units per frame, then the Wander calculation would be using a target location that is 10 units behind the agent as the reference point for calculating the new target location. That would certainly send the wandering agent along a very unrelaxing path.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/wander.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Wander behavior in action. The blue circle is the future circle. The yellow circle is the target circle. The small white circle is the rough target location. The yellow line from the center of the future circle to its edge points to the actual target location.</em>
    </figcaption>
</figure>

<h4 class="header3-swipe">Arrive</h4>

Arrive is similar to seek, except it slows the agent down as they approach the target location. 

Another good use of this behavior is to set the target location as the agent's current position. This makes them resistant to being pushed around, which was useful in my project as my physics system had no concept of friction. Without this, my agents would slide across the ground when bumped (assuming they had no targets to steer towards).

```cpp
static void Arrive(Vehicle& vehicle, Personality& data, const PhysicsBody& body)
{
  if (!data.m_has_target) return;

  // Self to target.
  glm::vec2 self_to_target{ data.m_target_position - body.GetPosition() };
  float distance_to_target{ glm::length(self_to_target) };

  // If closer than some epsilon, assume at target and set desired velocity to 0.
  if (distance_to_target > 0.00001f)
  {
    // Adjust speed. Reduce it if self is closer than slowing distance.
    float ramped_speed{ vehicle.GetMaxSpeed() * (distance_to_target / data.m_arrive.m_slowing_distance) };
    float clipped_speed{ fminf(ramped_speed, vehicle.GetMaxSpeed()) };

    // If desired is greater than current velocity, use Seek behavior / weight.
    // Otherwise, agent will accelerate to max speed much faster than normal,
    // sprinting towards the goal before slowing down.
    if (clipped_speed > glm::length(vehicle.GetVelocity()))
    {
      Seek(vehicle, data, body);
    }
    else
    {
      // Wanted velocity. If outside slowing distance, move at max_speed towards target.
      auto desired_velocity{ clipped_speed * (self_to_target / distance_to_target) };
      
      auto force{ desired_velocity - vehicle.GetVelocity() };
      vehicle.AddSteeringForce(data.m_arrive.m_weight * force);
    }
  }
  else
  {
    auto force{ -(vehicle.GetVelocity()) };
    vehicle.AddSteeringForce(data.m_arrive.m_weight * force);
  }
}
```

There are two interesting things happening in this code I want to point out:

First, if the agent gets close enough to the target location, I simply set the force to cancel out the remaining velocity. I use an awful hard-coded epsilon value for this. YOU should use a proper variable.

Second, in practice, this behavior will generally overshoot the target by a little bit unless the weight is set at least to the same value as the FPS of the physics loop. In my project, the physics system ran at 50 FPS and I achieved best results with Arrive by setting its weight to 50. But, while using a high value for the weight, the agent will move with a much higher acceleration than normal (compared to Seek) towards the target location until it becomes close enough that it begins to slow down.

To avoid this unwanted movement, I check if the `clipped_speed` (the desired speed based on distance to the target) is greater than the agent's current speed. If it is greater - meaning the agent wants to speed up - I used the Seek behavior to calculate the steering force. Otherwise, as long as the agent is wanting to slow down, I let Arrive calculate the steering force.

This works out nicely, as all behaviors share the same target info in the Personality struct.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/arrive.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Arrive behavior in action. The white square surrounded by the white circle are both the selected point on the map and the target location for the behavior. The orange circle is the stopping distance - when the target location is within this circle the agent will begin to slow down.</em>
    </figcaption>
</figure>

<h4 class="header2-swipe">Flocking & Neighbors</h4>

The next 3 behaviors are commonly enabled together to create a combined behavior called *flocking*. Flocking, like all other steering behavior concepts I've covered, comes from Craig Reynolds. You may have heard the term "boids" before, which is the name of a simulation he created showing the flocking behavior of birds using the combination of these simple behaviors.

An important step for the next behaviors (called Separate, Align, and Gather) is the evaluation of multiple neighbors. In Flee and Evade we saw neighbor searching as well, but that was something additional I added and you could conceivably rewrite them to use a single target location or agent. But for these next 3 behaviors, you cannot remove the idea of neighboring agents from them.

I mentioned in Flee and Evade that looping over every agent just to find the ones within a certain range is bad. I had to find a better method for neighbor searching for the flocking behaviors and ended up using spatial partitionig with a kd-tree library, [nanoflann](https://github.com/jlblancoc/nanoflann).

When the kd-tree performs a nearest neighbor search, nanoflann allows you to customize the evaluation of the neighbors. I use this to both return only neighbors that match a layer and limit the number of neighbors returned; for the layer comparison, I allow either an exact match (`wanted_layer & other_layer == wanted_layer`) or an overlapping match (`wanted_layer & other_layer != 0`). All of these are parameters exposed in the Personality struct. 

As part of the custom evaulation of the neighbors, I also confirm that their distance away is greater than 0. This means I do not have to check for `nan` values after normalizing any vector between the current agent and the neighbor agent, since I know that none of the neighboring agents I evaluate will be exactly at the current agent's position.

The final thing to mention is that an agent usually has a viewing angle - that is, an agent is only aware of the neighbors *that it can see*. I chose to do this check within the behavior itself, after collecting the neighbors from the kd-tree. This means that sometimes the agent will evaluate less than the maximum neighbors it was expecting. The alternative is to move that logic into the nearest neighbor search. There's no reason I settled on keeping this logic within the behavior - I simply didn't do any tests comparing the differences.

The viewing angle is exposed in the Personality struct as well.

I won't show a gif of the next 3 behaviors acting individually, but will show what flocking can look like after describing them all.

<h4 class="header3-swipe">Separate</h4>

The Separate behavior keeps the agent away from other agents by moving away from all neighbors, weighed by the inverse distance to each (so the agent moves further away the close a neighbor is).

```cpp
static void Separate(Entity me, Vehicle& vehicle, Personality& data, SB_KD_Tree& kd_tree)
{
  glm::vec2 total_separate_force{ 0.0f, 0.0f };

  // Find neighbors in radius.
  std::vector<nanoflann::ResultItem<uint32_t, float>> neighbors_data;
  neighbors_data.reserve(data.m_max_neighbor_count);
  {
    float squared_distance{ data.m_separate.m_distance * data.m_separate.m_distance };
    MyCustomResultSet<float, uint32_t> result_set{ 
      kd_tree.dataset_,                   // The pointcloud with some agent data associated with each point
      squared_distance,                   // Distance to look (this kd-tree uses distance squared in its calculations)
      neighbors_data,                     // Will be filled with the results (pointcloud index and distance)
      data.m_max_neighbor_count,          // Total neighbors to return
      data.m_separate.m_layer_list,       // Layer to match
      data.m_separate.m_use_exact_match   // If layer should be matched exactly or with overlap
    };
    nanoflann::SearchParameters params;
    params.sorted = false;
    kd_tree.findNeighbors(result_set, glm::value_ptr(vehicle.GetPosition()), params);
  }

  // Use group to get neighbor entity layer and vehicle data.
  auto group{ ECS().Registry.group<Layer, Vehicle>() };

  // Evaluate each neighbor.
  for (size_t i{ 0 }; i < neighbors_data.size(); ++i)
  {
    entt::entity neighbor_entity{ kd_tree.dataset_.m_data[neighbors_data[i].first].m_entity };
    if (neighbor_entity == me) continue;

    auto [layer, other_vehicle] = group.get(neighbor_entity);
  
    // Confirm in viewing range.
    auto vec_to_neighbor{ other_vehicle.GetPosition() - vehicle.GetPosition() };
    auto vec_to_neighbor_norm{ glm::normalize(vec_to_neighbor) };
    auto view_range_to_neighbor{ glm::dot(vehicle.GetOrientation(), vec_to_neighbor_norm) };
    if (view_range_to_neighbor <= data.m_viewing_angle) continue;

    // Adjust the velocity to get away from the other body by its distance away.
    float distance_to_other{ glm::distance(vehicle.GetPosition(), other_vehicle.GetPosition()) };
    float weight{ 1.0f / distance_to_other };
    auto direction_away_from_other{ glm::normalize(vehicle.GetPosition() - other_vehicle.GetPosition()) };
    
    // Calculate force.
    total_separate_force += direction_away_from_other * weight;
  }

  vehicle.AddSteeringForce(data.m_separate.m_weight * total_separate_force);
}
```

The first third of this is doing a nearest neighbor search with nanoflann.

The EnTT ECS allows the creation of groups, which sorts arrays of different components by entities to accelerate lookup speeds. So if the current entity's Layer component is at index 9 in the Layer component array, its Vehicle component will also be at index 9 in the Vehicle component array.

<h4 class="header3-swipe">Align</h4>

Align will make an agent try to match its velocity with the average velocity of its neighbors.

```cpp
static void Align(Entity me, Vehicle& vehicle, Personality& data, SB_KD_Tree& kd_tree)
{
  glm::vec2 total_align_force{ 0.0f, 0.0f };

  float desired_distance2{ data.m_align.m_distance * data.m_align.m_distance };
  float neighbors_counted{ 0.0f };

  // Find neighbors in radius.
  std::vector<nanoflann::ResultItem<uint32_t, float>> neighbors_data;
  neighbors_data.reserve(data.m_max_neighbor_count);
  {
    MyCustomResultSet<float, uint32_t> result_set{
      kd_tree.dataset_,
      desired_distance2,
      neighbors_data,
      data.m_max_neighbor_count,
      data.m_align.m_layer_list.Get(),
      data.m_align.m_use_exact_match
    };

    nanoflann::SearchParameters params;
    params.sorted = false;
    kd_tree.findNeighbors(result_set, glm::value_ptr(vehicle.GetPosition()), params);
  }

  // Use group to get neighbor entity layer and vehicle data.
  auto group{ ECS().Registry.group<Layer, Vehicle>() };

  // Evaluate each neighbor.		
  for (size_t i{ 0 }; i < neighbors_data.size(); ++i)
  {
    entt::entity neighbor_entity{ kd_tree.dataset_.m_data[neighbors_data[i].first].m_entity };
    if (neighbor_entity == me) continue;

    auto [layer, other_vehicle] = group.get(neighbor_entity);
    
    // Confirm in viewing range.
    auto vec_to_neighbor{ other_vehicle.GetPosition() - vehicle.GetPosition() };
    auto vec_to_neighbor_norm{ glm::normalize(vec_to_neighbor) };
    auto view_range_to_neighbor{ glm::dot(vehicle.GetOrientation(), vec_to_neighbor_norm) };
    if (view_range_to_neighbor <= data.m_viewing_angle) continue;

    neighbors_counted += 1.0f;
    total_align_force += other_vehicle.GetOrientation();	
  }

  if (neighbors_counted > 0.0f)
  {
    auto average_alignment{ total_align_force / neighbors_counted };
    auto desired_alignment{ glm::normalize(average_alignment) - vehicle.GetOrientation() };

    if (!std::isnan(desired_alignment.x))
    {
      vehicle.AddSteeringForce(data.m_align.m_weight * desired_alignment);
    }				
  }
}
```

Luckily, the kd-tree search bit stays almost the same for all of the flocking behaviors.

We must track how many neighbors were actually counted because we don't want to include the current agent in the average of the neighboring agent velocities.

In addition, we make a check for `nan` here because there's no guarantee that the average velocity is not the current agent's velocity. Better safe than sorry here.

<h4 class="header3-swipe">Gather</h4>

The final behavior for flocking is Gather. An agent will move towards another agent that is within their Gather range. If Separate is also on, they will eventually be pushed away and eventually find a happy equilibrium.

```cpp
static void Gather(Entity, Vehicle& vehicle, Personality& data, SB_KD_Tree& kd_tree)
{
  glm::vec2 total_gather_force{ 0.0f, 0.0f };

  float desired_distance2{ data.m_gather.m_distance * data.m_gather.m_distance };
  float neighbors_counted{ 0.0f };

  // Find neighbors in radius.
  std::vector<nanoflann::ResultItem<uint32_t, float>> neighbors_data;
  neighbors_data.reserve(data.m_max_neighbor_count);
  {
    MyCustomResultSet<float, uint32_t> result_set{
      kd_tree.dataset_,
      desired_distance2,
      neighbors_data,
      data.m_max_neighbor_count,
      data.m_gather.m_layer_list.Get(),
      data.m_gather.m_use_exact_match
    };

    nanoflann::SearchParameters params;
    params.sorted = false;
    kd_tree.findNeighbors(result_set, glm::value_ptr(vehicle.GetPosition()), params);
  }

  // Use group to get neighbor entity layer and vehicle data.
  auto group{ ECS().Registry.group<Layer, Vehicle>() };

  // Evaluate each neighbor.		
  for (size_t i{ 0 }; i < neighbors_data.size(); ++i)
  {
    entt::entity neighbor_entity{ kd_tree.dataset_.m_data[neighbors_data[i].first].m_entity };

    auto [layer, other_vehicle] = group.get(neighbor_entity);
    
    // Confirm in viewing range.
    auto vec_to_neighbor{ other_vehicle.GetPosition() - vehicle.GetPosition() };
    auto vec_to_neighbor_norm{ glm::normalize(vec_to_neighbor) };
    auto view_range_to_neighbor{ glm::dot(vehicle.GetOrientation(), vec_to_neighbor_norm) };
    if (view_range_to_neighbor <= data.m_viewing_angle) continue;

    neighbors_counted += 1.0f;
    total_gather_force += other_vehicle.GetPosition();    
  }

  // Always add self to the forces.
  neighbors_counted += 1.0f;
  total_gather_force += vehicle.GetPosition();

  if (neighbors_counted > 1.0f)
  {
    auto center_of_mass{ total_gather_force / neighbors_counted };
    auto desired_velocity{ glm::normalize(center_of_mass - vehicle.GetPosition()) * vehicle.GetMaxSpeed() };

    if (!std::isnan(desired_velocity.x))
    {
      vehicle.AddSteeringForce(data.m_gather.m_weight * desired_velocity);
    }
  }
}
```

In this behavior, we want to include the current agent in the calculations for the center of mass, and thus only want to do the final bit of math if at least 1 neighbor was found.

We make a check for `nan` because the current agent could very well be at the center of mass.

In practice, I had a difficult time balancing this behavior with the rest. Generally, I had to set the weight much lower than expected, but this made Gather sluggish (as you would expect). But leaving the weight equal to Separate's weight causes agents to rush towards each other. Values inbetween give varying results, and depending on where the agent is and where the center of mass is the movement can veer from normal to abnormal. I suspect the calculation for `desired_velocity` is the problem line, but would need to play around with it more to have an idea of what exactly needs to be done.

<h4 class="header3-swipe">Separate, Align, & Gather: Flocking</h4>

As promised, here's what flocking looked like in my project. In addition to flocking, the agents also have the Seek and Flee behaviors enabled.

<figure style="flex: 1; margin-top: 0px; text-align: center;">
    <img src="/assets/media/behaviors/flock_1.gif" type="gif" style="border: 1px white solid; max-width: 100%;">
    <figcaption>
        <em>Flocking behavior in action. Separate (orange circles), Align (small purple circles), and Gather (green circles). Agents flee (large purple circles) from the player, who is also at the location of their Seek target (white square). As the agents move away from the target, the cluster into groups based on their nearest neighbors.</em>
    </figcaption>
</figure>