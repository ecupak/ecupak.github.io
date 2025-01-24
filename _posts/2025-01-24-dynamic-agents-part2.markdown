---
layout: post
title:  "Dynamic Agents using Steering Behaviors and Flow Fields in C++ [2/3]"
date:   2025-01-24 13:55:45 +0100
categories: jekyll update
---

<h3 class="header1-swipe">Introduction</h3>

My recent student project at BUas (Breda University of Applied Science) covered steering behaviors and flow fields. 

The final product was a system that allowed agents to easily enable and disable the behaviors to use as well as adjust the settings of those behaviors. In addition, via ImGui you could adjust those same settings and see their effects during runtime - either on a per-agent basis or for all agents of a class.

I want to cover the ideas behind what I learned as well as share the code I implemented. I have divided this into 3 parts / posts. This is part 2 of 3:

- Part 1 covers the basics of steering behaviors, how they work with the physics system, and the C++ implementation of 9 steering behaviors (Seek, Flee, Pursue, Evade, Arrive, Wander, Separate, Align, and Gather).

- **Part 2** covers the basics of flow fields, how to construct one, the C++ implementation of a working flow field, how to add bilinear interpolation, and how to handle dynamic objects moving across the flow field.

- Part 3 covers how to combine the steering behavior and flow fields into a steering system, how to modify agents during runtime with new behaviors, and how the steering system and agents communicate.

I will explain everything within a 2D system / world. 

For my project, [EnTT](https://github.com/skypjack/entt) (an entity component system (ECS)) was utilized and was instrumental in putting all the pieces together; I have not considered the architecture for an engine without an ECS.

I also used [glm](https://github.com/g-truc/glm) as my math library. You'll see it in the C++ code snippets but should be able to easily swap it out for whatever you prefer.

<br/>

<h3 class="header1-swipe">Part 2: Flow Fields</h3>

Craig Reynolds' paper does mention *Flow Field Following* as a behavior, but I did not follow his description very closely in my own implementation. His paper says to use the future position of the agent, which I do not do; I simply use the agent's current position.

The rest of my knowledge about building and using flow fields comes from scattered resources online.

A really excellent paper on flow fields is [Emerson's paper, "Crowd Pathfinding and Steering Using Flow Field Tiles"](https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter23_Crowd_Pathfinding_and_Steering_Using_Flow_Field_Tiles.pdf). It covers a lot of interesting topics, such as using multiple cost fields to represent different units' movement abilities as well as handling large maps by partitioning the flow fields and using a search method like A\* to identify the fields that need to be updated. I highly suggest reading it if you find any of this interesting.

<h4 class="header2-swipe">How They Work</h4>

A flow field is a grid where each cell contains a vector. An agent will lookup their position on the grid and use the vector at that cell as their desired velocity.

To create the vectors, first a cell is marked as the taget (or goal). The field is then constructed so that each cell has a vector that points towards the neighboring cell that is closest to the goal.

The power of a flow field is that once a goal is marked, all agents can use the same flow field to lookup their desired velocity. This is advantageous when you have thousands of agents - instead of performing individual pathfinding for each agent you can calculate the flow field once and have all the agents use it.

<h4 class="header3-swipe">Cost and Integration Fields</h4>

A flow field ends up with vectors pointing towards a goal, but we need to take a few steps back to see how that gets created.

First, there are 2 other fields involved with creating a flow field - a cost field and an integration field. They are both the same size as the flow field (or rather, the integration field and flow field are both the same size as the cost field). 
- A cost field holds data about how easy it is to pass through each cell.
- An integration field is a temporary field that contains the best path from every cell to the goal.

When a cell is marked as the goal, the following steps happen:
1. The integration field is constructed using Dijkstra's algorithm, starting at the goal cell. It uses the cost field for the cost of each cell.
2. The flow field loops over every cell in the integration field and checks its 8 neighboring cells to find the neighbor with the lowest value. Then it assigns a vector in the flow field cell pointing to the cell with the lowest value.

<h4 class="header3-swipe">Dijkstra's Algorithm </h4>

Dijkstra's algorithm finds the shortest path between a source and destination node. In a flow field, a node is a cell. Normally the search would end after the destination node is reached. But in our case, we won't assign any destination. Instead, the algorithm will spread out from the source (the cell marked as the goal) until it has reached all the cells on the grid.

As it visits each cell it will assign a value to that cell; cells with a lower value were reached earlier than cells with a higher value. The value of a cell is determined by the value of that same cell on the cost field as well as the cost of all the cells on the path between it and the goal.

For a much more detailed explanation of Dijkstra's algorithm, I recommend [this RedBlobGames article](https://www.redblobgames.com/pathfinding/a-star/introduction.html#dijkstra).

<h4 class="header3-swipe">Construction of A Flow Field</h4>

To construct a flow field, we first need a cost field. This represents the data of the world. Each cell holds a value that determines how difficult it is to travel across (ranging from 1 to 254 - easiest to hardest). A value of 0 is reserved for the goal cell and a value of 255 is reserved for obstacles that cannot be passed through.

Consider the cost field below. It is mostly grass with a few rocks in the center and a hill next to the rocks:

<table>
  <tbody>
    <tr>
      <td>1</td>
      <td>1</td>
      <td>1</td>
      <td>1</td>
    </tr>
    <tr>
      <td>1</td>
      <td>255</td>
      <td>90</td>
      <td>1</td>
    </tr>
    <tr>
      <td>1</td>
      <td>255</td>
      <td>255</td>
      <td>1</td>
    </tr>
    <tr>
      <td>1</td>
      <td>1</td>
      <td>1</td>
      <td>1</td>
    </tr>
  </tbody>
</table>
<br/>

If we select the bottom-right cell as the goal, Dijkstra's algorithm will create an integration field that looks like this:

<table>
  <tbody>
    <tr>
      <td>6</td>
      <td>5</td>
      <td>4</td>
      <td>3</td>
    </tr>
    <tr>
      <td>5</td>
      <td>260</td>
      <td>92</td>
      <td>2</td>
    </tr>
    <tr>
      <td>4</td>
      <td>257</td>
      <td>256</td>
      <td>1</td>
    </tr>
    <tr>
      <td>3</td>
      <td>2</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>
<br/>

Starting at any cell, you can find the shortest path to the goal by repeatedly moving to the neighboring cell with the lowest value. We can use that logic to build the flow field. For each cell, find the neighbor (including diagonals) that has the lowest value and assign a vector to that cell pointing that way (the goal cell is assigned a `(0, 0)` vector):

<table style="text-align: center;">
  <tbody>
    <tr>
      <td style="color: orange;">&#129122;</td>
      <td>&#129122;</td>
      <td>&#129126;</td>
      <td>&#129123;</td>
    </tr>
    <tr>
      <td>&#129123;</td>
      <td style="color: orange;">&#129125;</td>
      <td>&#129126;</td>
      <td>&#129123;</td>
    </tr>
    <tr>
      <td>&#129126;</td>
      <td>&#129126;</td>
      <td>&#129126;</td>
      <td>&#129123;</td>
    </tr>
    <tr>
      <td>&#129122;</td>
      <td>&#129122;</td>
      <td>&#129122;</td>
      <td></td>
    </tr>
  </tbody>
</table>
<br/>

The cells with an orange vector each had 2 neighbors with the same value. In this case, it doesn't matter which neighbor is selected.

Normally, for cells with impassable cost values (255), a vector of `(0, 0)` would be assigned. That would give us a flow field that looks like this:

<table style="text-align: center;">
  <tbody>
    <tr>
      <td>&#129122;</td>
      <td>&#129122;</td>
      <td>&#129122;</td>
      <td>&#129123;</td>
    </tr>
    <tr>
      <td>&#129123;</td>
      <td></td>
      <td>&#129126;</td>
      <td>&#129123;</td>
    </tr>
    <tr>
      <td>&#129126;</td>
      <td></td>
      <td></td>
      <td>&#129123;</td>
    </tr>
    <tr>
      <td>&#129122;</td>
      <td>&#129122;</td>
      <td>&#129122;</td>
      <td></td>
    </tr>
  </tbody>
</table>
<br/>

But, with larger fields, it's very normal to have cells with an integration value of 255 or greater that are **not** obstacles - they are just either very far from the goal or have a lot of high-cost cells along the path between them and the goal. This would cause the flow field to incorrectly mark those cells as impassable.

Instead, the integration field should assign the max possible value to any cell whose cost field value is 255 *and stop searching from that node*. This does limit the size of the flow field so that this max value cannot normally happen. This would give us the same result as the previous flow field, but the integration field would look like this instead:

<table>
  <tbody>
    <tr>
      <td>6</td>
      <td>5</td>
      <td>4</td>
      <td>3</td>
    </tr>
    <tr>
      <td>5</td>
      <td>MAX</td>
      <td>92</td>
      <td>2</td>
    </tr>
    <tr>
      <td>4</td>
      <td>MAX</td>
      <td>MAX</td>
      <td>1</td>
    </tr>
    <tr>
      <td>3</td>
      <td>2</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>
<br/>

<h4 class="header2-swipe">Flow Field Implementation</h4>

I designed the flow fields in my project to contain their associated cost field and integration field. Although the integration field is a temporary structure, it was easier to make it a persistent object that I reused when needed. These are stored as member variables named `m_cfield` and `m_ifield`.

The grid for each field is a vector of ints. Given an `(x, y)` grid coordinate, the vector index is obtained by the formula `index = x + y * grid_width`.

The most important pubic methods of the flow field are `SetGoal()` and `Lookup()`; the most important private method is `Build()`. We'll look at these in detail.

```cpp
bool FlowField::SetGoal(const glm::vec2& goal_position)
{
  // Return invalid index if goal is not within field.
  if (!IsInBounds(goal_position)) return false;

  // Find the cell that contains the goal position and store it.
  auto goal_grid_pos{ GetGridCoords(goal_position) };
  m_goal = goal_grid_pos;

  // Build the flow field.
  Build();
  m_is_dirty = false;

  return true;
}
```

`IsInBounds()` does just what you'd expect. My flow fields track their position in world space and I use this information to confirm the goal is on the field.

`GetGridCoords()` converts a world position into a grid coordinate. There is a reverse method as well, `GetWorldPosition()`, that takes a grid coordinate as the argument.

The other 2 variables, `m_goal` and `m_is_dirty`, are used when dealing with dynamic objects which is discussed later. We can ignore them for now.

The next method to look at is `Build()`. I need to show a few helpers first, so that you can make sense of what's happening.

I have an premade array of the relative directions of each neighbor:

```cpp
const std::vector<glm::ivec2> Directions::all_dirs
{
  glm::ivec2{ -1,  1 },
  glm::ivec2{  0,  1 },
  glm::ivec2{  1,  1 },

  glm::ivec2{ -1,  0 },
  glm::ivec2{  0,  0 },
  glm::ivec2{  1,  0 },

  glm::ivec2{ -1, -1 },
  glm::ivec2{  0, -1 },
  glm::ivec2{  1, -1 }
};

const std::vector<glm::ivec2>& Directions::getAllDirs() { return all_dirs; }
```

In addition, my flow field doesn't store the actual vector in each cell. It stores an index that can be used to get the vector. The index is stored as an enum:

```cpp
enum Direction : uint8_t
{
  NW  = 0,
  N,
  NE,

  W,
  C, // zero vector
  E,

  SW,
  S,
  SE,
};
```

And for completeness, the vector array that the above enum indexes into (a static variable of the flow field):

```cpp
const glm::vec2 FlowField::directions[9]{
  glm::normalize(glm::vec2{-1.0f,  1.0f}),
  glm::normalize(glm::vec2{ 0.0f,  1.0f}),
  glm::normalize(glm::vec2{ 1.0f,  1.0f}),

  glm::normalize(glm::vec2{-1.0f,  0.0f}),
                 glm::vec2{ 0.0f,  0.0f},
  glm::normalize(glm::vec2{ 1.0f,  0.0f}),

  glm::normalize(glm::vec2{-1.0f, -1.0f}),
  glm::normalize(glm::vec2{ 0.0f, -1.0f}),
  glm::normalize(glm::vec2{ 1.0f, -1.0f})
};
```


Now we can look at `Build()` with that out of the way:

```cpp
void FlowField::Build()
{
  // Build integration field with goal, using cost field.
  auto& goal_grid_position{ m_goal };
  m_ifield.Build(goal_grid_position.x, goal_grid_position.y, m_cfield);

  // Neighbors to check.
  auto& all_directions{ Directions::getAllDirs() };

  // Initial lowest value to beat is the max value possible.
  size_t initial_lowest_value{ static_cast<size_t>(-1) };

  // Check all cells and set the vector based on which neighbor has the lowest integration value.
  for (int y{ 0 }; y < m_grid_size.y; ++y)
  {
    for (int x{ 0 }; x < m_grid_size.x; ++x)
    {
      int& cell_value{ m_field[x + y * m_grid_size.x] };

      // If impassable, mark with zero vector.
      if (auto integration_value{ m_ifield.Lookup(x, y) }; 
        integration_value == IntegrationField::Marker::Obstacle || integration_value == IntegrationField::Marker::Unchecked)
      {
        cell_value = Direction::C;
      }

      // Point towards neighbor cell with lowest value.
      else
      {
        size_t lowest_value{ initial_lowest_value };
        uint8_t neighbor_idx{ 0 };
        uint8_t vector_idx{ 0 };

        for (auto& dir : all_directions)
        {
          int neighbor_x{ x + dir.x };
          int neighbor_y{ y + dir.y };

          // Only check neighbors that are in-bounds.
          if (neighbor_x >= 0 && neighbor_x < m_grid_size.x && neighbor_y >= 0 && neighbor_y < m_grid_size.y)
          {
            // Store lowest value.						
            auto neighbor_cost{ m_ifield.Lookup(neighbor_x, neighbor_y) };
            if (neighbor_cost < lowest_value)
            {
              lowest_value = neighbor_cost;
              vector_idx = neighbor_idx;
            }
          }

          ++neighbor_idx;
        }

        // If a passable tile was found, use its relative index as the vector lookup index.
        if (lowest_value != initial_lowest_value)
        {
          cell_value = vector_idx;
        }
        else
        {
          cell_value = Direction::C;
        }
      }
    }
  }
}
```

The first thing that happens is the building of the integration field. I'll cover that in the next section, but let's continue to look at what happens after that.

As I loop over all the cells in the integration field, I stop early if the cell is an obstacle or it was not reached. A cell can end up not being reached by Dijkstra's algorithm if it is surrounded by cells that are impassable.

Otherwise, the neighbors of the cell are checked to see which one has the lowest value. I designed it so that the order that the neighbors were evaulated in was the same order as the vector lookup indices:
- The first neighbor checked (index 0) is to the left and above `(-1, 1)`.
- The vector index of 0 is mapped to the enum NW (northwest).
- The vector at index 0 is `glm::normalize(glm::vec2{-1.0f,  1.0f})`.

An important thing to note is that cell (0, 0) is the bottom-left of the flow field grid. Rows are visually *above* the previous row. So when `y` increases, we are moving *up* to the row above. This was a decision I had to make due to the coordinate system my engine used (where `(0, 0)` was the center of the map, with `y` increasing as you went towards the top of the screen).

After checking all neighbors, I assign the found vector index to the cell. If no neighbors beat the initial lowest value, I assign a zero vector to the cell.

The last method to look at for the flow field is `Lookup()`:

```cpp
glm::vec2 FlowField::Lookup(const glm::vec2& agent_position)
{
  // Return zero vector if position is not within field.
  if (!IsInBounds(agent_position)) return directions[Direction::C];

  // If field is 'dirty' (cost field has changed since last build), rebuild it.
  if (m_is_dirty)
  {
    Build();
    m_is_dirty = false;
  }
  
  // Convert coordinates to grid coordiantes and find vector at cell.
  auto cell_index{ GetCellIndex(GetGridCoords(agent_position)) };
  return directions[m_field[cell_index]];
}
```

This is short and sweet. The mid-section is dealing with dynamic objects, but if the field is dirty we rebuild it and mark it as clean.

`GetCellIndex()` is a helper method that takes grid coordinates and retuns the result of `x + y * grid_width`.

And finally, the index at the wanted flow field cell is used to return the correct vector.

<h4 class="header2-swipe">Integration Field Implementation</h4>

The integration field is only used when the flow field is being (re)built. As we saw above, the first step when building the flow field is to build the integration field.

Just like how the flow field got relative vectors to all the neighbors, the integration field does almost the same. But instead of checking all 8 neighbors, only the cardinal neighbors are checked.

The `Build()` method for the integration field is essentially Dijkstra's algorithm:

```cpp
void IntegrationField::Build(const int destination_x, const int destination_y, CostField& cfield)
{
  struct Cell
  {
    Cell() = default;

    Cell(int cost_so_far, int x, int y)
      : m_cost_so_far{ cost_so_far }
      , m_x{ x }
      , m_y{ y }
      , m_visited{ false }
    {	}

    int m_cost_so_far{ -1 };
    int m_x{ 0 };
    int m_y{ 0 };
    bool m_visited{ false };
  };

  // Fill map with unchecked values. If unvisited, it will keep this value.
  std::fill(m_field.begin(), m_field.end(), Marker::Unchecked);

  // Unique cells that have been put in queue.
  std::unordered_map<unsigned int, Cell> cells;

  // Cell that is the destination.
  Cell first_cell{ 0, destination_x, destination_y };
  int index{ destination_x + destination_y * m_grid_size.x };
  cells[index] = first_cell;

  // Create queue and seed with destination cell.
  PairedPriorityQueue<unsigned int, unsigned int> frontier;
  frontier.Put(index, 0);

  // Cardinal directions (N/W/E/S).
  const auto& cardinal_dirs{ Directions::getCardinalDirs() };

  // Explore map until all cells visited.
  while (!frontier.IsEmpty())
  {
    // Get top cell.
    // Lowest cost cell will be on top.
    auto parent_index{ frontier.Get() };
    auto& parent{ cells[parent_index] };

    // If cell was already visited, skip it.
    if (parent.m_visited) continue;

    // Check all neighbors.
    for (const auto& dir : cardinal_dirs)
    {
      // Get coordiantes.
      int x{ parent.m_x + dir.x };
      int y{ parent.m_y + dir.y };

      // Skip if out of bounds.
      if (x < 0 || x >= m_grid_size.x || y < 0 || y >= m_grid_size.y) continue;

      // Get cost field value.
      // If neighbor cannot be passed, mark as obstacle and end this branch early.
      auto neighbor_cost_value{ cfield.lookup(x, y) };
      if (neighbor_cost_value == CostField::Marker::NoPass)
      {
        m_field[x + y * m_grid_size.x] = Marker::Obstacle;
        continue;
      }

      // Pull stored integration field data.
      int neighbor_index{ x + y * m_grid_size.x };
      auto& neighbor{ cells[neighbor_index] };

      // Set x and y every time since we did not check to see if the cell already existed.
      neighbor.m_x = x;
      neighbor.m_y = y;

      // If neighbor cost has not been set, or new path is cheaper, update it.
      if (neighbor.m_cost_so_far == -1 || (parent.m_cost_so_far + neighbor_cost_value) < neighbor.m_cost_so_far)
      {
        neighbor.m_cost_so_far = parent.m_cost_so_far + neighbor_cost_value;
      }

      // If the neighbor has NOT been visited, add to queue.
      if (!neighbor.m_visited)
      {
        frontier.Put(neighbor_index, neighbor.m_cost_so_far);
      }			
    }

    // Cell has been visited.			
    parent.m_visited = true;

    // In this fill method, once visited no faster path to cell will be found. Set cost.
    m_field[parent_index] = parent.m_cost_so_far;
  }
}
```

The `PairedPriorityQueue` is a wrapper for `std::priority_queue`:

```cpp
template <typename TItemValue, typename TPriorityValue>
class PairedPriorityQueue
{
public:
  PairedPriorityQueue() = default;

  void Put(const TItemValue item, const TPriorityValue priority)
  { 
      m_elements.emplace(priority, item); 
  }
  
  TItemValue Get()
  {
      TItemValue top_item{ m_elements.top().second };
      m_elements.pop();

      return top_item;
  }
  
  bool IsEmpty() 
  { 
      return m_elements.empty(); 
  }

private:
  using queue_element = std::pair<TPriorityValue, TItemValue>;
  std::priority_queue<queue_element, std::vector<queue_element>, std::greater<queue_element>> m_elements;
};
```

This is heavily inspired by [the priority queue from RedBlobGames](https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp-queue-with-priorities).

<h4 class="header2-swipe">As A Steering Behavior</h4>

Now that we know how to create a flow field and update it with new goals, we need to allow agents to use this as a behavior.

The flow field will be owned by the steering system, so the lookup happens outside of the usual behavior logic. The logic itself, once the vector is known, is the simplest out of all the behaviors:

```cpp
static void FlowFieldFollowing(Vehicle& vehicle, Personality& data, const glm::vec2& flow_field_force)
{
  auto force{ flow_field_force - vehicle.GetVelocity() };
  vehicle.AddSteeringForce(data.m_flow_field_following.m_weight * force);
}
```

<h4 class="header2-swipe">Bilinear Interpolation</h4>

Right now the flow field only gives 1 of 8 vectors to follow. This can make the agents seem a little too robotic and look especially strange as they cross the border between flow field cells with different vectors: the agent will immediately snap to moving in a new direction - not smooth at all.

To fix this issue, we can add bilinear interpolation to the mix. This will allow the agent to sample the vectors of the 4 closest flow field cells and weigh them based on how close the agent is to each cell, then take the average.

These images show the difference between a flow field without (left) and with (right) bilinear interpolation:

<div style="display: flex; flex: 1 0 auto; flex-flow: row;">
    <div style="display: flex; flex: 1 0 auto; flex-flow: column; max-width: 48%;">
        <img class="custom scaling150" src="/assets/media/bilinear_off.png" type="png" style="flex: 0 1 auto;">
    </div>
    <span style="min-width: 4%;"></span>
    <div style="display: flex; flex: 1 0 auto; flex-flow: column; max-width: 48%;">
        <img class="custom scaling150" src="/assets/media/bilinear_on.png" type="png" style="flex: 0 1 auto;">
    </div>
</div>

The blue lines are the vector of each flow field cell and extend from the center of each cell. The white squares are the positions being sampled - each is in the upper-right section of the cell. The green lines are the returned vector from the flow field for each position.

On the left, the green and blue lines for each cell are the same - there is no interpolation.

On the right, the green lines are an average of the vectors of the 4 nearest cells, weighed based on the distance from the white square to the center of each cell.

My implementation was guided by [this GeekforGeeks article](https://www.geeksforgeeks.org/what-is-bilinear-interpolation/), which I found especially helpful.

I'm going to present this in the context of the `Lookup()` method, as that is where it belongs:

```cpp
glm::vec2 FlowField::Lookup(const glm::vec2& agent_position, bool use_bilinear)
{
  // Return zero vector if position is not within field.
  if (!IsInBounds(agent_position)) return directions[Direction::C];

  // If field is 'dirty' (cost field has changed since last build), rebuild it.
  if (m_is_dirty)
  {
    Build();
    m_is_dirty = false;
  }
    
  if (use_bilinear)
  {
    // Get floating grid coordinates.
    // (Can treat as fractional value along grid -> 2.9 is cell 2 and 90% to cell 3).
    glm::vec2 pos{ (agent_position + m_offset) / m_cell_size };

    // Add 0.5 to value to find maximum interpolation cell.
    // Keep value as interpolation key position that all other cells will calculate from.
    // Will interpolate between this cell and the cells that are 1 less on the x or y axis and the cell that is 1 less on both the x and y axis.
    pos += 0.5f;
    glm::ivec2 key_pos{ static_cast<glm::ivec2>(pos) };

    // Find min and max offsets from key_pos.
    // Used in case any cell is outside of grid boundary (imaginary cell). Will make imaginary cell use adjacent cell value instead.
    glm::ivec2 min_offset{ -1, -1 };
    glm::ivec2 max_offset{ 0, 0 };

    if (key_pos.x == 0) { min_offset.x = 0; }
    if (key_pos.y == 0) { min_offset.y = 0; }

    if (key_pos.x == m_grid_size.x) { max_offset.x = -1; }
    if (key_pos.y == m_grid_size.y) { max_offset.y = -1; }

    // Get values of Q11, Q21, Q12, and Q22 cells.
    // If cell is not on grid, repeat value from previous cell.
    glm::ivec2 q11_pos{ key_pos + min_offset };

    glm::ivec2 q21_pos{
      key_pos.x + max_offset.x,
      key_pos.y + min_offset.y
    };

    glm::ivec2 q12_pos{
      key_pos.x + min_offset.x,
      key_pos.y + max_offset.y
    };

    glm::ivec2 q22_pos{ key_pos + max_offset };

    // Build bilinear interpolation formula:
    // x interpolation key = c term; y interpolation key = d term.
    // 1 - c = a term; 1 - d = b term.
    float c{ pos.x - static_cast<float>(key_pos.x) };
    float d{ pos.y - static_cast<float>(key_pos.y) };
    float a{ 1.0f - c };
    float b{ 1.0f - d };

    // Calculate.
    auto q11{ directions[m_field[GetCellIndex(q11_pos)]] };
    auto q21{ directions[m_field[GetCellIndex(q21_pos)]] };
    auto q12{ directions[m_field[GetCellIndex(q12_pos)]] };
    auto q22{ directions[m_field[GetCellIndex(q22_pos)]] };

    return (q11 * a * b) + (q21 * c * b) + (q12 * a * d) + (q22 * c * d);
  }
  else
  {
    // Convert coordinates to grid coordiantes and find vector at cell.
    auto cell_index{ GetCellIndex(GetGridCoords(agent_position)) };
    return directions[m_field[cell_index]];
  }
}
```

I find the top-right cell relative to the agent and use that as the "key cell". I then only need to subtract 1 on each axis to get the other cells that make up the quartet of nearby cells to evaulate.

If any cell is out-of-bounds, I reuse the nearest cell on the border as a placeholder for the imaginary out-of-bounds cell.

<h4 class="header2-swipe">Dynamic Objects</h4>

Handling dynamic objects allows your flow field to be reactive to the world. But even with my method I found it tedious to work with.

The approach I used was:
1. Store the cost field separately after its creation, without any dynamic objects on it (only the static data of the world).
2. When an object is moved (or during the first frame), make a copy of the cost field and update all the cells that the object overlaps.
3. Replace the cost field of the flow field with the newly updated cost field.
4. Mark the flow field as dirty.

Even with multiple dynamic objects this approach still works, as I would check all the objects in 1 pass to see which ones moved, and then in a second pass I would add each one to the copied cost field.

It's important that you keep the original cost field "clean" and do not save any dynamic object data to it. Otherwise you will have no way to correctly "erase" them from the map. Even if you know their previous positions, you won't know what the previous value of the cost field was (unless you are only working with values of 1 and 255 and can guarantee that the objects will never overlap a static obstacle).

By marking the flow field as dirty, it will rebuild itself the next time a lookup is performed.

Here's an example of how this looked in my project:

<img src="/assets/media/dynamic_boxes.gif" type="gif" style="border: 1px white solid; max-width: 100%;">

When checking which cells the object overlapped in the cost field, I contracted the object's corners by half of the width of each flow field cell. This meant that the object didn't block a cell unless it reached the middle of that cell. This avoided situations where an object barely on a cell would block the entire cell.
<br/>

<h3 class="header1-swipe">Part 2 Conclusion</h3>

We’ve now covered the creation and updating of flow fields.

Flow fields require:
- A cost field, whose cells represent how difficult it is to pass through the world at a given location.
- An integration field, whose cells represent the shortest path from any cell to the goal cell.
- The flow field, whose cells contain a vector (or vector index) towards the neighboring cell that is along the shortest path to the goal.

Flow fields can be used in steering behaviors as easily as the other 9 behaviors discussed in Part 1.

Movement across a flow field can be smoothed by using bilinear interpolation.

Flow fields can represent dynamic objects in the world as well by updating an intermediary cost field with the cells that are overlapped by each object and assigning this to the flow field.

The next post will cover the steering system, the glue that holds everything together.

<h3 class="header1-swipe">&nbsp;</h3>

<div style="text-align: center;">
  <img src="/assets/media/buas_logo.png" type="png" style="border: 1px white solid; max-width: 100%;">
</div>