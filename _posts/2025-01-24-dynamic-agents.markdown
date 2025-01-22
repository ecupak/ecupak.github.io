---
layout: post
title:  "Dynamic Agents using Steering Behaviors and Flow Fields in C++"
date:   2025-01-22 13:55:45 +0100
categories: jekyll update
---

<h3 class="header1-swipe">Introduction</h3>

My recent student project at BUas (Breda University of Applied Science) covered steering behaviors and flow fields. In this post I want to cover the ideas behind what I learned as well as share the code I implemented. I have divided this post into 3 parts:

- **Part 1** covers the basics of steering behaviors, how they work with the physics system, and the C++ implementation of 9 steering behaviors (seek, flee, pursue, evade, arrive, wander, separate, align, and gather).

* **Part 2** covers the basics of flow fields, how to construct one, the C++ implementation of a working flow field, how to add bilinear interpolation, and how to handle dynamic objects moving across the flow field.

- **Part 3** covers how to combine the steering behavior and flow fields into a steering system, how to modify agents during runtime with new behaviors, and how the steering system and agents communicate.

I will explain everything within a 2D system / world. 

For my project, [EnTT](https://github.com/skypjack/entt) (an entity component system (ECS)) was utilized and was instrumental in putting all the pieces together; I have not considered the architecture for an engine without an ECS. The value of Part 3 may vary wildly depending on if you are using an ECS or not.

<br/>

<h3 class="header1-swipe">Part 1: Steering Behaviors</h3>

A majority of my knowledge about steering behaviors comes from the [paper published by Craig Reynolds](https://www.red3d.com/cwr/steer/gdc99/). I encourage you to find time to read through it if you add steering behaviors to your game.

<h4 class="header2-swipe">How They Work</h4>

Every steering behavior works by calculating a steering force and applying it to the agent's velocity. Calculating the steering force will be covered later in the C++ implementation section. Let's first look at what we do once we have the steering force:
- The steering force is truncated by the agent's maximum allowed force.
- Acceleration is calculated and applied to the agent's velocity.
- The agent's velocity is then truncated by the agent's maximum allowed velocity.
- Finally, the agent's velocity is applied to their position.

You can easily imagine that the agent's maximum force and velocity might change depending on how they are moving around the world - running on foot, in a car, or on a horse. All these different methods of moving around will be contained in a Vehicle class (don't get too wrapped up in the name Vehicle - it's just a name to symbolize a method of moving).

The Vehicle class will hold information about the method of movement: max force, max speed, orientation (the forward vector), and any additional information that is related to movement (such as a turning radius). Using the bullet list above, my Vehicle class had the following method:

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

To finish out this section, I'll show what `Truncate()` does:

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

But it does assume the agent has a physics body with velocity, because the steering behaviors calculate steering forces which the Vehicle applies directly to the agent's velocity (as seen earlier).

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

One final thing to discuss before we can dive into my implementation of some steering behaviors is what I call the agent's *personality*. 

Every steering behavior has parameters that can be adjusted (for examlpe, the distance from the agent that it looks for other agents it will *flee* from - or maybe *pursue* instead). We don't want to hardcode these values into the steering behavior, so instead I created a Personality struct that holds all of these parameters and each agent has their own Personality.

The Personality struct will be discussed more in Part 3, but I wanted to introduce it now so that we would know about all of the variables used in the steering behaviors.

