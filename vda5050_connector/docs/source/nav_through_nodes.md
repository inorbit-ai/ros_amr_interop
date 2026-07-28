# NavThroughNodes Handler

## Overview

`NavThroughNodes` is a C++ plugin interface (base class) provided by the connector adapter. It is
the recommended navigation handler when the robot needs to traverse multiple nodes in a single
continuous goal — as opposed to `NavToNode`, which sends one node at a time.

When the VDA5050 controller receives an order it builds a *drivable segment*: all contiguous
released edge–node pairs up to the first HARD/SOFT blocking action. It sends this segment as a
single `NavigateThroughNodes` action goal to the adapter. The `NavThroughNodes` handler is
responsible for translating that goal into whatever motion primitives the robot understands.

```
Controller ──NavigateThroughNodes goal──► Adapter
                                              │
                                     NavThroughNodes handler
                                              │
                                      Robot motion stack
```

### VDA5050 sequence ID layout

VDA5050 §6.1.1 defines a shared sequence ID space for nodes and edges:

```
Node₀  Edge₁  Node₂  Edge₃  Node₄  …
```

Edge `S` connects Node `S-1` to Node `S+1`. The controller sends only the *target* nodes (not the
starting node), so the handler always receives equal-length `edges` and `nodes` vectors where
`edge[i]` leads to `node[i]`.

---

## Implementing a NavThroughNodes handler

### 1. Inherit from `adapter::NavThroughNodes`

```cpp
#include <vda5050_connector/nav_through_nodes.hpp>

class MyNavHandler : public adapter::NavThroughNodes
{
public:
  void configure() override;
  void execute() override;
  bool cancel() override;

protected:
  // Optional: react to in-flight path extension (see below)
  void onNavigationExtended(size_t old_edge_count) override;
};

PLUGINLIB_EXPORT_CLASS(MyNavHandler, adapter::NavThroughNodes)
```

### 2. Implement the three mandatory methods

| Method | Called when | Responsibility |
|---|---|---|
| `configure()` | Once at adapter startup | Create ROS publishers/subscribers/clients, read parameters, call `setupExtendNavigationService()` |
| `execute()` | Each time a new navigation goal arrives (edges/nodes already set by `reset()`; read them via `getNavigationSnapshot()`) | Start robot motion |
| `cancel()` | When a `cancelOrder` instant action is received | Stop the robot and return `true` on success |

Within all three methods the following inherited members are available:

| Member | Type | Description |
|---|---|---|
| `node_` | `rclcpp::Node*` | The adapter ROS node |
| `robot_name_` | `std::string` | Robot name (from parameter) |
| `goal_handle_` | `shared_ptr<GoalHandle>` | Active action goal handle |
| `feedback_` | `shared_ptr<Feedback>` | Feedback to publish |
| `result_` | `shared_ptr<Result>` | Result to send on completion |

The edge and node lists are **private**. Read them through the thread-safe accessor
`getNavigationSnapshot()`, which returns a `{edges, nodes}` copy taken under a shared lock:

```cpp
const auto [edges, nodes] = getNavigationSnapshot();
```

Do not cache references to the internal vectors: they are mutated under an exclusive lock by the
`ExtendNavigation` service (see below), so any retained reference would race with an in-flight
extension.

Use `goal_handle_->publish_feedback(feedback_)`, `goal_handle_->succeed(result_)` and
`goal_handle_->abort(result_)` to drive the action lifecycle. Populate `feedback_->last_node` with
the node the robot just passed before each feedback publish.

### 3. Register as a pluginlib plugin

In your package's `plugins.xml`:

```xml
<library path="my_nav_handler">
  <class type="MyNavHandler" base_class_type="adapter::NavThroughNodes">
    <description>My robot's navigate-through-nodes handler.</description>
  </class>
</library>
```

In `CMakeLists.txt`:

```cmake
pluginlib_export_plugin_description_file(vda5050_connector plugins.xml)
```

---

## In-flight goal extension (ExtendNavigation)

### Background

VDA5050 supports *stitching*: the TMS can extend a running order's base while the robot is still
driving, avoiding a stop-and-restart at the end of the current base. The controller handles this
via the `ExtendNavigation` ROS service.

**Flow:**

```
TMS ──stitch order──► Controller
                           │
              [navigation is active?]
                     yes  │
                           ▼
              ExtendNavigation service call
                           │
                    NavThroughNodes handler
                    (appends edges/nodes,
                     calls onNavigationExtended)
```

The service is named:

```
/<namespace>/<manufacturer>/<robot_name>/adapter/extend_navigation
```

### Setting up the service in your handler

Call `setupExtendNavigationService()` inside your `configure()` implementation, **after** `node_`
and `robot_name_` are available (they are set by the adapter before `configure()` is called):

```cpp
void MyNavHandler::configure()
{
  // ... create your publishers, subscribers, clients ...

  setupExtendNavigationService();   // registers the ExtendNavigation service
}
```

The base class handles the service callback automatically. It validates the request, verifies that
`request->nodes[0]` matches the last node already in `nodes_msg_`, appends the new edges and
nodes, then calls `onNavigationExtended()`.

### Reacting to the extension

Override `onNavigationExtended(size_t old_edge_count)` to extend your motion primitive in-flight:

```cpp
void MyNavHandler::onNavigationExtended(size_t old_edge_count)
{
  // Re-snapshot to observe the just-appended segments.
  const auto [edges, nodes] = getNavigationSnapshot();
  // edges/nodes now contain the new segments starting at old_edge_count.
  // Extend your planner, re-queue tasks, etc.
}
```

`old_edge_count` is the number of edges **before** the extension, so in the freshly taken
snapshot `edges[old_edge_count]` is the first newly added edge.

> **Contract — re-snapshot inside `onNavigationExtended()`.** The hook runs *after* the base
> class releases the navigation lock, so calling `getNavigationSnapshot()` from it is safe (no
> deadlock, no re-entrancy). A snapshot captured earlier in `execute()` is memory-safe but is
> frozen at goal start and will never observe the extension — relying on it makes stitching
> silently no-op. Always take a fresh snapshot here.

If `onNavigationExtended` is not overridden the base no-op is used: the new edges/nodes are
stored and will be dispatched normally once the current navigation finishes.

### ExtendNavigation service contract

```
# Request
vda5050_msgs/Edge[] edges    # new edges (len == len(nodes) - 1)
vda5050_msgs/Node[] nodes    # nodes[0] = stitch reference, nodes[1:] = new targets

# Response
bool   success
string message
```

The service returns `success=false` if:
- There is no active navigation goal.
- `edges` is empty or `nodes` has fewer than 2 entries.
- `len(edges) != len(nodes) - 1`.
- `nodes[0]` does not match the last node already known by the handler.

---
