# LSTE Scores Through One Runtime Example

This document explains `S_target`, `S_env`, `S_ctx`, and `S_total` by following
one `blue_cup` frame through the running ROS nodes. Each mathematical symbol is
introduced together with the concrete Python value it represents.

The runtime integration is in
[`score_adapter.py`](../src/lste_core/scripts/utils/score_adapter.py). The ROS
message definitions are in [`lste_msgs/msg`](../src/lste_msgs/msg).

All source references below use the line numbers in the current workspace as
of 2026-07-21.

## 1. Task Data In Memory

**Source:** [`lste_task_node.py:40-76`](../src/lste_core/scripts/lste_task_node.py#L40-L76)
loads the JSON fields into ROS; [`LsteTask.msg:1-15`](../src/lste_msgs/msg/LsteTask.msg#L1-L15)
defines their in-memory message types.

The task starts in `blue_cup.json`. `lste_task_node.py` converts it to an
`LsteTask` ROS message. In Python, its relevant fields look like this:

```python
task.target_name = "blue mug"

task.env_related_structures = [
    "desk",
    "computer monitor",
    "chair",
]

task.env_type_prior = [
    "office workspace",
    "shared working area",
]

task.obj_key_objects = [
    "monitor",
    "other mugs",
    "file folder",
    "chair",
]

task.obj_negative_clues = [
    "door",
    "fire hydrant",
    "fire extinguisher",
]

task.ctx_left = "yellow mug"
task.ctx_right = "computer monitor"
```

These are normal Python strings and lists generated from ROS fields such as
`string` and `string[]`. They are not NumPy vectors.

The task fields map to the symbols used later as follows:

| Symbol | Runtime value |
| --- | --- |
| `target_name` | `"blue mug"` |
| $R$ | `task.env_related_structures` |
| $E$ | `task.env_type_prior` |
| $K$ | `task.obj_key_objects` |
| $N$ | `task.obj_negative_clues` |
| $C$ | `[task.ctx_left, task.ctx_right]` |

`layout_prior` and `search_strategy` remain inside `task.raw_json`. They do not
become separate `LsteTask` fields and are not used by the score calculation.

## 2. One Example Detection Frame

**Source:** [`lste_det_node.py:228-359`](../src/lste_core/scripts/lste_det_node.py#L228-L359)
runs both GroundingDINO prompts, converts tensors to Python lists, constructs
`LsteDetection` objects, and publishes `/lste/detections`.
[`LsteDetection.msg:1-7`](../src/lste_msgs/msg/LsteDetection.msg#L1-L7) and
[`LsteDetections.msg:1-8`](../src/lste_msgs/msg/LsteDetections.msg#L1-L8) define
the stored fields.

Suppose GroundingDINO publishes this `LsteDetections` message for one camera
frame. The notation below is Python-like but has the same fields as the actual
ROS message:

```python
detections.prompt_b_terms = [
    "desk",
    "computer monitor",
    "chair",
    "yellow mug",
    "door",
    "fire hydrant",
    "fire extinguisher",
]

detections.target_dets = [
    LsteDetection(
        label="blue mug",
        score=0.82,
        cx=0.50,
        cy=0.50,
        w=0.10,
        h=0.14,
    ),
]

detections.env_dets = [
    LsteDetection("yellow mug",      0.74, 0.62, 0.50, 0.08, 0.12),
    LsteDetection("computer monitor", 0.91, 0.38, 0.48, 0.20, 0.18),
    LsteDetection("desk",            0.88, 0.50, 0.70, 0.60, 0.20),
    LsteDetection("chair",           0.83, 0.82, 0.55, 0.16, 0.24),
    LsteDetection("door",            0.72, 0.95, 0.50, 0.08, 0.60),
]
```

Each detection stores:

```text
label, confidence, center_x, center_y, width, height
```

The box numbers are normalized image coordinates in `[0, 1]`. For example,
the blue mug center is halfway across and halfway down the image.

## 3. `S_target`: Is The Target Visible?

### What the symbols look like

**Source:** [`score_adapter.py:68-77`](../src/lste_core/scripts/utils/score_adapter.py#L68-L77)
converts `LsteDetection[]` into the `(label, score)` pairs shown below.

The code tokenizes the target name:

```python
target_tokens = ["blue", "mug"]  # mathematical symbol T
```

It then examines the target detections as `(label, score)` pairs:

```python
target_pairs = [
    ("blue mug", 0.82),
]
```

In the formula, a candidate $d_i$ is one entry from this list, and $q_i$ is its
confidence. In this frame:

```text
d_1 = "blue mug"
q_1 = 0.82
```

### Runtime calculation

**Source:** [`calc_s_target.py:130-161`](../src/lste_core/scripts/scoring/calc_s_target.py#L130-L161)
implements tokenization, the 60% partial-match rule, and maximum-confidence
selection. [`score_adapter.py:121-133`](../src/lste_core/scripts/utils/score_adapter.py#L121-L133)
adapts its return value to the ROS runtime.

A label qualifies when it contains the complete target name or at least 60%
of the target-name tokens. The number of required token matches is:

```python
required_hits = max(1, ceil(0.6 * len(target_tokens)))
              = max(1, ceil(0.6 * 2))
              = 2
```

`"blue mug"` contains both required tokens, so it is a candidate. The code
takes the largest confidence among all candidates:

```python
candidate_scores = [0.82]
s_target = max(candidate_scores)
         = 0.82
detected = True
```

This is what the compact formula means:

$$
S_{target}=\max_i(q_i)=0.82.
$$

If no label matches, the stored result is `s_target = 0.0` and
`detected = False`.

## 4. `S_env`: Does The Scene Look Promising?

### Building the positive terms

**Source:** [`score_adapter.py:97-109`](../src/lste_core/scripts/utils/score_adapter.py#L97-L109)
concatenates the four sources. [`calc_s_env.py:107-116`](../src/lste_core/scripts/scoring/calc_s_env.py#L107-L116)
normalizes them into the dictionary shown below.

The code concatenates four Python lists:

```python
all_positive_terms = (
    detections.prompt_b_terms
    + task.env_related_structures
    + task.obj_key_objects
    + task.env_type_prior
)
```

After lowercase normalization and exact deduplication, the result is stored as
a dictionary whose keys are the terms:

```python
positive_terms = {                  # mathematical symbol P
    "desk": "desk",
    "computer monitor": "computer monitor",
    "chair": "chair",
    "yellow mug": "yellow mug",
    "door": "door",
    "fire hydrant": "fire hydrant",
    "fire extinguisher": "fire extinguisher",
    "monitor": "monitor",
    "other mugs": "other mugs",
    "file folder": "file folder",
    "office workspace": "office workspace",
    "shared working area": "shared working area",
}
```

Therefore, $P$ in the mathematics is this dictionary, and $|P|$ is its length:

```python
len(positive_terms) == 12
```

The repeated `chair` from two task fields is stored only once.

### Building the negative terms

**Source:** [`score_adapter.py:112-118`](../src/lste_core/scripts/utils/score_adapter.py#L112-L118)
extracts `obj_negative_clues`; the same normalization function at
[`calc_s_env.py:107-116`](../src/lste_core/scripts/scoring/calc_s_env.py#L107-L116)
deduplicates them.

Negative clues are normalized in the same way:

```python
negative_terms = {                  # mathematical symbol N
    "door": "door",
    "fire hydrant": "fire hydrant",
    "fire extinguisher": "fire extinguisher",
}

len(negative_terms) == 3
```

### Matching detections

**Source:** [`score_adapter.py:136-161`](../src/lste_core/scripts/utils/score_adapter.py#L136-L161)
performs substring matching and calculates the two coverage ratios.

For `S_env`, $D$ means the concrete list of environment-detection labels:

```python
env_labels = [                      # mathematical symbol D
    "yellow mug",
    "computer monitor",
    "desk",
    "chair",
    "door",
]
```

The code uses lowercase substring matching. The resulting dictionaries are:

```python
detected_positive = {
    "yellow mug": 0.0,
    "computer monitor": 0.0,
    "monitor": 0.0,   # also matched by "computer monitor"
    "desk": 0.0,
    "chair": 0.0,
    "door": 0.0,
}

detected_negative = {
    "door": 0.0,
}
```

The `0.0` values are placeholders. Only the dictionary keys and their counts
are used. Detector confidences such as `0.91` and `0.72` do not affect
`S_env`.

The coverage ratios are ordinary divisions:

```python
pos_ratio = len(detected_positive) / len(positive_terms)
          = 6 / 12
          = 0.5

neg_ratio = len(detected_negative) / len(negative_terms)
          = 1 / 3
          = 0.333
```

These two ratios are the mathematical symbols $r_{pos}$ and $r_{neg}$.

### Turning coverage into `S_env`

**Source:** [`calc_s_env.py:19-29`](../src/lste_core/scripts/scoring/calc_s_env.py#L19-L29)
implements `logistic01`; [`score_adapter.py:163-167`](../src/lste_core/scripts/utils/score_adapter.py#L163-L167)
applies the positive score, negative penalty, and clamp. The active parameter
values are launched at
[`run_nodes_tmux.sh:164-169`](../scripts/lifecycle/run_nodes_tmux.sh#L164-L169).

The runtime passes each ratio through a sigmoid helper:

```python
pos_score = logistic01(
    x=0.5,
    midpoint=0.25,
    steepness=6,
)
# approximately 0.818

neg_score = logistic01(
    x=0.333,
    midpoint=0.15,
    steepness=12,
)
# approximately 0.900
```

The helper computes `1 / (1 + exp(-steepness * (x - midpoint)))`, except that
an input of exactly `0` returns `0` and an input of exactly `1` returns `1`.

The final Python arithmetic is:

```python
raw_env = pos_score - 0.7 * neg_score
        = 0.818 - 0.7 * 0.900
        = approximately 0.187

s_env = clamp(raw_env, -1.0, 1.0)
      = 0.187
```

If fewer than two entries exist in `detections.env_dets`, the function returns
`s_env = 0.0`, `pos_ratio = 0.0`, and `neg_ratio = 0.0` immediately.

## 5. `S_ctx`: Are The Expected Neighbors Near The Target?

### Context list in memory

**Source:** [`calc_s_ctx.py:145-176`](../src/lste_core/scripts/scoring/calc_s_ctx.py#L145-L176)
normalizes `ctx_left` and `ctx_right` into `ctx_terms`.

The two task fields become a Python list:

```python
ctx_terms = [                       # mathematical symbol C
    "yellow mug",
    "computer monitor",
]
```

The code removes stopwords, truncates each term to three words, removes terms
containing the target name, and removes duplicates.

Despite their JSON names, `left` and `right` are not checked against actual
left/right image positions. They are treated as two required neighbor labels.

### Box vectors in memory

**Source:** [`score_adapter.py:46-94`](../src/lste_core/scripts/utils/score_adapter.py#L46-L94)
converts `(cx, cy, w, h)` ROS fields into `[x1, y1, x2, y2]` lists and wraps
them as context entries. [`calc_s_ctx.py:134-142`](../src/lste_core/scripts/scoring/calc_s_ctx.py#L134-L142)
computes centers and Euclidean distances.

An `LsteDetection` stores center-width-height fields. The adapter converts each
one into a four-float Python list:

```python
# [x1, y1, x2, y2]
blue_box = [0.45, 0.43, 0.55, 0.57]
yellow_box = [0.58, 0.44, 0.66, 0.56]
monitor_box = [0.28, 0.39, 0.48, 0.57]
```

These lists are the box vectors written as $b_t$ and $b_j$ in the mathematics.
They are plain `list[float]` values, not learned feature vectors.

The helper extracts their two-float centers:

```python
target_center = (0.50, 0.50)
yellow_center = (0.62, 0.50)
monitor_center = (0.38, 0.48)
```

For every neighbor, it stores a distance beside the original detection:

```python
distances = [
    (0.120, yellow_detection),
    (0.122, monitor_detection),
    (0.200, desk_detection),
    (0.324, chair_detection),
    (0.450, door_detection),
]
```

Here, each distance is the ordinary Euclidean distance between two image
centers. This list is sorted from nearest to farthest.

### Coverage in this frame

**Source:** [`calc_s_ctx.py:179-227`](../src/lste_core/scripts/scoring/calc_s_ctx.py#L179-L227)
sorts neighbors, calculates the top-30% cutoff, matches context labels, and
counts `top30_hits`.

There are five neighbors, so the closest 30% contains two entries:

```python
top30_cutoff = max(1, ceil(0.3 * len(distances)))
             = max(1, ceil(0.3 * 5))
             = 2
```

Both required context terms occur in those first two entries:

```python
top30_hits = 2
ctx_count = 2
ctx_coverage = top30_hits / ctx_count
             = 2 / 2
             = 1.0
```

This stored `ctx_coverage` value is the symbol $r_{coverage}$.

### Proximity in this frame

**Source:** [`calc_s_ctx.py:186-242`](../src/lste_core/scripts/scoring/calc_s_ctx.py#L186-L242)
normalizes distance, calculates proximity and coverage averages, applies the
`-0.4`/`0.35` special cases, and combines the context score.
[`score_adapter.py:170-220`](../src/lste_core/scripts/utils/score_adapter.py#L170-L220)
handles invalid context and selects the best target box.

Each distance is first divided by the maximum normalized-image distance,
`sqrt(2)`. It is then converted so that distance zero gives proximity `1` and
distance `sqrt(2) * 0.5` or greater gives proximity `0`:

```python
yellow_distance_normalized = 0.120 / sqrt(2)  # 0.0849
yellow_proximity = 1 - 0.0849 / 0.5           # 0.830

monitor_distance_normalized = 0.122 / sqrt(2) # 0.0863
monitor_proximity = 1 - 0.0863 / 0.5          # 0.827

ctx_proximity = (0.830 + 0.827) / 2
              = 0.829
```

The stored `ctx_proximity` value is the symbol $r_{proximity}$.

The context score averages coverage and proximity:

```python
s_ctx = 0.5 * ctx_coverage + 0.5 * ctx_proximity
      = 0.5 * 1.0 + 0.5 * 0.829
      = 0.915
```

Special runtime cases are:

- no context term detected: `s_ctx = -0.4`;
- two context terms configured but only one detected: cap `s_ctx` at `0.35`;
- no configured context, no target detection, or no neighbor: `s_ctx = -1.0`;
- multiple target boxes: calculate each one and keep the largest `s_ctx`.

Detection confidence is stored in each context entry but is not used by this
calculation.

## 6. `S_total`: Combining The Three Results

**Source:** [`score_adapter.py:223-260`](../src/lste_core/scripts/utils/score_adapter.py#L223-L260)
calls all three score methods and builds the result. The weighted sum and clamp
are implemented by
[`aggregate_target_score.py:131-138`](../src/lste_core/scripts/scoring/aggregate_target_score.py#L131-L138).

At this point, the runtime has a small Python dictionary:

```python
scores = {
    "target": 0.820,
    "env": 0.187,
    "ctx": 0.915,
}

weights = {
    "target": 0.2,
    "env": 0.3,
    "ctx": 0.5,
}
```

The final arithmetic is:

```python
raw_total = (
    0.2 * scores["target"]
    + 0.3 * scores["env"]
    + 0.5 * scores["ctx"]
)
# 0.2 * 0.820 + 0.3 * 0.187 + 0.5 * 0.915
# = 0.678

s_total = clamp(raw_total, 0.0, 1.0)
        = 0.678
```

The score node publishes these values in an `LsteScores` ROS message:

```python
output.detected = True
output.s_target = 0.820
output.s_env = 0.187
output.s_ctx = 0.915
output.s_total = 0.678
output.pos_ratio = 0.500
output.neg_ratio = 0.333
output.ctx_coverage = 1.000
output.ctx_proximity = 0.829
```

All numeric fields in `LsteScores.msg` are stored as ROS `float32` values.
The message layout is defined at
[`LsteScores.msg:1-14`](../src/lste_msgs/msg/LsteScores.msg#L1-L14), and the
fields are assigned and published at
[`lste_score_node.py:84-99`](../src/lste_core/scripts/lste_score_node.py#L84-L99).

## 7. Runtime Data Flow

```text
blue_cup.json
    |
    v
LsteTask ROS message
    |
    +--------------------------+
    |                          |
    v                          v
prompt generation       task fields cached by score node
    |                          |
    v                          |
GroundingDINO                   |
    |                          |
    v                          |
LsteDetections ROS message ----+
    |
    v
Python lists/dictionaries in score_adapter.py
    |
    +--> S_target = 0.820
    +--> S_env    = 0.187
    +--> S_ctx    = 0.915
    |
    v
LsteScores ROS message: S_total = 0.678
```

Each camera frame produces a new `LsteDetections` message and a new set of
scores. The state node then evaluates the score stream over time; its time
windows are separate from the per-frame calculations described here.

## 8. Important Current Behaviors

**Source for the prompt overlap issue:**
[`lste_prompt_node.py:103-126`](../src/lste_core/scripts/lste_prompt_node.py#L103-L126)
forces context and negative terms into `prompt_B`; `score_adapter.py:97-109`
then includes all `prompt_B` terms in the positive term list.

- `related_structures`, `key_objects`, `env_type_prior`, and `prompt_B` become
  one positive dictionary with no source-specific weights.
- `S_target` uses detector confidence. `S_env` and `S_ctx` do not.
- Substring matching means `computer monitor` can satisfy both `computer
  monitor` and `monitor`.
- The prompt node forces `target_ctx` and `negative_clues` into `prompt_B`.
  Since every `prompt_B` term is added to the positive dictionary, `door` in
  this example is both a positive and negative term. This is a current
  implementation flaw, not the intended semantics.
- `S_ctx = -1` represents missing context evidence but is still included as a
  numeric penalty when `S_total` is calculated.
