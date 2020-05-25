#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

/// Alias for `f64`.
using Real = double;

/// Stores the state for sub-node `limit`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Limit {
  /// Input: `limit.usr.x`
  Real svar_x;
  /// Output: `limit.usr.y`
  Real svar_y;
};

/// Prints usage.
/// Handles CLA.
/// Alias for `i64`.
using Int = int64_t;

/// Stores the state for sub-node `findlocation_p`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Int |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Findlocation_p {
  /// Input: `findlocation_p.usr.x`
  Real svar_x;
  /// Output: `findlocation_p.usr.y`
  Int svar_y;
};

/// Stores the state for sub-node `findlocation_d`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Int |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Findlocation_d {
  /// Input: `findlocation_d.usr.x`
  Real svar_x;
  /// Output: `findlocation_d.usr.y`
  Int svar_y;
};

/// Stores the state for sub-node `rule_base`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Int |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Rule_base {
  /// Input: `rule_base.usr.x`
  Int svar_x;
  /// Output: `rule_base.usr.y`
  Real svar_y;
};

/// Stores the state for sub-node `y1calc`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `low` | Real |
/// | `high` | Real |
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Y1calc {
  /// Input: `y1calc.usr.low`
  Real svar_low;
  /// Input: `y1calc.usr.high`
  Real svar_high;
  /// Input: `y1calc.usr.x`
  Real svar_x;
  /// Output: `y1calc.usr.y`
  Real svar_y;
};

/// Stores the state for sub-node `y1fuzzify`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Int |
/// | `input` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `y1calc` | [Y1calc](struct.Y1calc.html) | `abs_9`, `abs_11`, `input` | `abs_12` | [fuzzy.lus line 208](../src/lus/fuzzy.lus.html#208) |
/// | `y1calc` | [Y1calc](struct.Y1calc.html) | `abs_7`, `abs_9`, `input` | `abs_10` | [fuzzy.lus line 207](../src/lus/fuzzy.lus.html#207) |
/// | `y1calc` | [Y1calc](struct.Y1calc.html) | `abs_5`, `abs_7`, `input` | `abs_8` | [fuzzy.lus line 206](../src/lus/fuzzy.lus.html#206) |
/// | `y1calc` | [Y1calc](struct.Y1calc.html) | `abs_3`, `abs_5`, `input` | `abs_6` | [fuzzy.lus line 205](../src/lus/fuzzy.lus.html#205) |
/// | `y1calc` | [Y1calc](struct.Y1calc.html) | `abs_1`, `abs_3`, `input` | `abs_4` | [fuzzy.lus line 204](../src/lus/fuzzy.lus.html#204) |
/// | `y1calc` | [Y1calc](struct.Y1calc.html) | `abs_0`, `abs_1`, `input` | `abs_2` | [fuzzy.lus line 203](../src/lus/fuzzy.lus.html#203) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Y1fuzzify {
  /// Input: `y1fuzzify.usr.x`
  Int svar_x;
  /// Input: `y1fuzzify.usr.input`
  Real svar_input;
  /// Output: `y1fuzzify.usr.y`
  Real svar_y;
  /// Local, call: `y1fuzzify.res.abs_12`
  Real svar_abs_12;
  /// Local, invisible local: `y1fuzzify.res.abs_11`
  Real svar_abs_11;
  /// Local, call: `y1fuzzify.res.abs_10`
  Real svar_abs_10;
  /// Local, invisible local: `y1fuzzify.res.abs_9`
  Real svar_abs_9;
  /// Local, call: `y1fuzzify.res.abs_8`
  Real svar_abs_8;
  /// Local, invisible local: `y1fuzzify.res.abs_7`
  Real svar_abs_7;
  /// Local, call: `y1fuzzify.res.abs_6`
  Real svar_abs_6;
  /// Local, invisible local: `y1fuzzify.res.abs_5`
  Real svar_abs_5;
  /// Local, call: `y1fuzzify.res.abs_4`
  Real svar_abs_4;
  /// Local, invisible local: `y1fuzzify.res.abs_3`
  Real svar_abs_3;
  /// Local, call: `y1fuzzify.res.abs_2`
  Real svar_abs_2;
  /// Local, invisible local: `y1fuzzify.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `y1fuzzify.res.abs_0`
  Real svar_abs_0;
  /// Call to `y1calc` ([fuzzy.lus line 203](../src/lus/fuzzy.lus.html#203)).
  Y1calc y1calc_5;
  /// Call to `y1calc` ([fuzzy.lus line 204](../src/lus/fuzzy.lus.html#204)).
  Y1calc y1calc_4;
  /// Call to `y1calc` ([fuzzy.lus line 205](../src/lus/fuzzy.lus.html#205)).
  Y1calc y1calc_3;
  /// Call to `y1calc` ([fuzzy.lus line 206](../src/lus/fuzzy.lus.html#206)).
  Y1calc y1calc_2;
  /// Call to `y1calc` ([fuzzy.lus line 207](../src/lus/fuzzy.lus.html#207)).
  Y1calc y1calc_1;
  /// Call to `y1calc` ([fuzzy.lus line 208](../src/lus/fuzzy.lus.html#208)).
  Y1calc y1calc_0;
};

/// Stores the state for sub-node `y2calc`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `low` | Real |
/// | `high` | Real |
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Y2calc {
  /// Input: `y2calc.usr.low`
  Real svar_low;
  /// Input: `y2calc.usr.high`
  Real svar_high;
  /// Input: `y2calc.usr.x`
  Real svar_x;
  /// Output: `y2calc.usr.y`
  Real svar_y;
};

/// Stores the state for sub-node `y2fuzzify`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Int |
/// | `input` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `y2calc` | [Y2calc](struct.Y2calc.html) | `abs_9`, `abs_11`, `input` | `abs_12` | [fuzzy.lus line 225](../src/lus/fuzzy.lus.html#225) |
/// | `y2calc` | [Y2calc](struct.Y2calc.html) | `abs_7`, `abs_9`, `input` | `abs_10` | [fuzzy.lus line 224](../src/lus/fuzzy.lus.html#224) |
/// | `y2calc` | [Y2calc](struct.Y2calc.html) | `abs_5`, `abs_7`, `input` | `abs_8` | [fuzzy.lus line 223](../src/lus/fuzzy.lus.html#223) |
/// | `y2calc` | [Y2calc](struct.Y2calc.html) | `abs_3`, `abs_5`, `input` | `abs_6` | [fuzzy.lus line 222](../src/lus/fuzzy.lus.html#222) |
/// | `y2calc` | [Y2calc](struct.Y2calc.html) | `abs_1`, `abs_3`, `input` | `abs_4` | [fuzzy.lus line 221](../src/lus/fuzzy.lus.html#221) |
/// | `y2calc` | [Y2calc](struct.Y2calc.html) | `abs_0`, `abs_1`, `input` | `abs_2` | [fuzzy.lus line 220](../src/lus/fuzzy.lus.html#220) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Y2fuzzify {
  /// Input: `y2fuzzify.usr.x`
  Int svar_x;
  /// Input: `y2fuzzify.usr.input`
  Real svar_input;
  /// Output: `y2fuzzify.usr.y`
  Real svar_y;
  /// Local, call: `y2fuzzify.res.abs_12`
  Real svar_abs_12;
  /// Local, invisible local: `y2fuzzify.res.abs_11`
  Real svar_abs_11;
  /// Local, call: `y2fuzzify.res.abs_10`
  Real svar_abs_10;
  /// Local, invisible local: `y2fuzzify.res.abs_9`
  Real svar_abs_9;
  /// Local, call: `y2fuzzify.res.abs_8`
  Real svar_abs_8;
  /// Local, invisible local: `y2fuzzify.res.abs_7`
  Real svar_abs_7;
  /// Local, call: `y2fuzzify.res.abs_6`
  Real svar_abs_6;
  /// Local, invisible local: `y2fuzzify.res.abs_5`
  Real svar_abs_5;
  /// Local, call: `y2fuzzify.res.abs_4`
  Real svar_abs_4;
  /// Local, invisible local: `y2fuzzify.res.abs_3`
  Real svar_abs_3;
  /// Local, call: `y2fuzzify.res.abs_2`
  Real svar_abs_2;
  /// Local, invisible local: `y2fuzzify.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `y2fuzzify.res.abs_0`
  Real svar_abs_0;
  /// Call to `y2calc` ([fuzzy.lus line 220](../src/lus/fuzzy.lus.html#220)).
  Y2calc y2calc_5;
  /// Call to `y2calc` ([fuzzy.lus line 221](../src/lus/fuzzy.lus.html#221)).
  Y2calc y2calc_4;
  /// Call to `y2calc` ([fuzzy.lus line 222](../src/lus/fuzzy.lus.html#222)).
  Y2calc y2calc_3;
  /// Call to `y2calc` ([fuzzy.lus line 223](../src/lus/fuzzy.lus.html#223)).
  Y2calc y2calc_2;
  /// Call to `y2calc` ([fuzzy.lus line 224](../src/lus/fuzzy.lus.html#224)).
  Y2calc y2calc_1;
  /// Call to `y2calc` ([fuzzy.lus line 225](../src/lus/fuzzy.lus.html#225)).
  Y2calc y2calc_0;
};

/// Stores the state for sub-node `dy1calc`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `low` | Real |
/// | `high` | Real |
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Dy1calc {
  /// Input: `dy1calc.usr.low`
  Real svar_low;
  /// Input: `dy1calc.usr.high`
  Real svar_high;
  /// Input: `dy1calc.usr.x`
  Real svar_x;
  /// Output: `dy1calc.usr.y`
  Real svar_y;
};

/// Stores the state for sub-node `dy1fuzzify`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Int |
/// | `input` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `dy1calc` | [Dy1calc](struct.Dy1calc.html) | `abs_9`, `abs_11`, `input` | `abs_12` | [fuzzy.lus line 247](../src/lus/fuzzy.lus.html#247) |
/// | `dy1calc` | [Dy1calc](struct.Dy1calc.html) | `abs_7`, `abs_9`, `input` | `abs_10` | [fuzzy.lus line 246](../src/lus/fuzzy.lus.html#246) |
/// | `dy1calc` | [Dy1calc](struct.Dy1calc.html) | `abs_5`, `abs_7`, `input` | `abs_8` | [fuzzy.lus line 245](../src/lus/fuzzy.lus.html#245) |
/// | `dy1calc` | [Dy1calc](struct.Dy1calc.html) | `abs_3`, `abs_5`, `input` | `abs_6` | [fuzzy.lus line 244](../src/lus/fuzzy.lus.html#244) |
/// | `dy1calc` | [Dy1calc](struct.Dy1calc.html) | `abs_1`, `abs_3`, `input` | `abs_4` | [fuzzy.lus line 243](../src/lus/fuzzy.lus.html#243) |
/// | `dy1calc` | [Dy1calc](struct.Dy1calc.html) | `abs_0`, `abs_1`, `input` | `abs_2` | [fuzzy.lus line 242](../src/lus/fuzzy.lus.html#242) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Dy1fuzzify {
  /// Input: `dy1fuzzify.usr.x`
  Int svar_x;
  /// Input: `dy1fuzzify.usr.input`
  Real svar_input;
  /// Output: `dy1fuzzify.usr.y`
  Real svar_y;
  /// Local, call: `dy1fuzzify.res.abs_12`
  Real svar_abs_12;
  /// Local, invisible local: `dy1fuzzify.res.abs_11`
  Real svar_abs_11;
  /// Local, call: `dy1fuzzify.res.abs_10`
  Real svar_abs_10;
  /// Local, invisible local: `dy1fuzzify.res.abs_9`
  Real svar_abs_9;
  /// Local, call: `dy1fuzzify.res.abs_8`
  Real svar_abs_8;
  /// Local, invisible local: `dy1fuzzify.res.abs_7`
  Real svar_abs_7;
  /// Local, call: `dy1fuzzify.res.abs_6`
  Real svar_abs_6;
  /// Local, invisible local: `dy1fuzzify.res.abs_5`
  Real svar_abs_5;
  /// Local, call: `dy1fuzzify.res.abs_4`
  Real svar_abs_4;
  /// Local, invisible local: `dy1fuzzify.res.abs_3`
  Real svar_abs_3;
  /// Local, call: `dy1fuzzify.res.abs_2`
  Real svar_abs_2;
  /// Local, invisible local: `dy1fuzzify.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `dy1fuzzify.res.abs_0`
  Real svar_abs_0;
  /// Call to `dy1calc` ([fuzzy.lus line 242](../src/lus/fuzzy.lus.html#242)).
  Dy1calc dy1calc_5;
  /// Call to `dy1calc` ([fuzzy.lus line 243](../src/lus/fuzzy.lus.html#243)).
  Dy1calc dy1calc_4;
  /// Call to `dy1calc` ([fuzzy.lus line 244](../src/lus/fuzzy.lus.html#244)).
  Dy1calc dy1calc_3;
  /// Call to `dy1calc` ([fuzzy.lus line 245](../src/lus/fuzzy.lus.html#245)).
  Dy1calc dy1calc_2;
  /// Call to `dy1calc` ([fuzzy.lus line 246](../src/lus/fuzzy.lus.html#246)).
  Dy1calc dy1calc_1;
  /// Call to `dy1calc` ([fuzzy.lus line 247](../src/lus/fuzzy.lus.html#247)).
  Dy1calc dy1calc_0;
};

/// Stores the state for sub-node `dy2calc`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `low` | Real |
/// | `high` | Real |
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Dy2calc {
  /// Input: `dy2calc.usr.low`
  Real svar_low;
  /// Input: `dy2calc.usr.high`
  Real svar_high;
  /// Input: `dy2calc.usr.x`
  Real svar_x;
  /// Output: `dy2calc.usr.y`
  Real svar_y;
};

/// Stores the state for sub-node `dy2fuzzify`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Int |
/// | `input` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `dy2calc` | [Dy2calc](struct.Dy2calc.html) | `abs_9`, `abs_11`, `input` | `abs_12` | [fuzzy.lus line 266](../src/lus/fuzzy.lus.html#266) |
/// | `dy2calc` | [Dy2calc](struct.Dy2calc.html) | `abs_7`, `abs_9`, `input` | `abs_10` | [fuzzy.lus line 265](../src/lus/fuzzy.lus.html#265) |
/// | `dy2calc` | [Dy2calc](struct.Dy2calc.html) | `abs_5`, `abs_7`, `input` | `abs_8` | [fuzzy.lus line 264](../src/lus/fuzzy.lus.html#264) |
/// | `dy2calc` | [Dy2calc](struct.Dy2calc.html) | `abs_3`, `abs_5`, `input` | `abs_6` | [fuzzy.lus line 263](../src/lus/fuzzy.lus.html#263) |
/// | `dy2calc` | [Dy2calc](struct.Dy2calc.html) | `abs_1`, `abs_3`, `input` | `abs_4` | [fuzzy.lus line 262](../src/lus/fuzzy.lus.html#262) |
/// | `dy2calc` | [Dy2calc](struct.Dy2calc.html) | `abs_0`, `abs_1`, `input` | `abs_2` | [fuzzy.lus line 261](../src/lus/fuzzy.lus.html#261) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Dy2fuzzify {
  /// Input: `dy2fuzzify.usr.x`
  Int svar_x;
  /// Input: `dy2fuzzify.usr.input`
  Real svar_input;
  /// Output: `dy2fuzzify.usr.y`
  Real svar_y;
  /// Local, call: `dy2fuzzify.res.abs_12`
  Real svar_abs_12;
  /// Local, invisible local: `dy2fuzzify.res.abs_11`
  Real svar_abs_11;
  /// Local, call: `dy2fuzzify.res.abs_10`
  Real svar_abs_10;
  /// Local, invisible local: `dy2fuzzify.res.abs_9`
  Real svar_abs_9;
  /// Local, call: `dy2fuzzify.res.abs_8`
  Real svar_abs_8;
  /// Local, invisible local: `dy2fuzzify.res.abs_7`
  Real svar_abs_7;
  /// Local, call: `dy2fuzzify.res.abs_6`
  Real svar_abs_6;
  /// Local, invisible local: `dy2fuzzify.res.abs_5`
  Real svar_abs_5;
  /// Local, call: `dy2fuzzify.res.abs_4`
  Real svar_abs_4;
  /// Local, invisible local: `dy2fuzzify.res.abs_3`
  Real svar_abs_3;
  /// Local, call: `dy2fuzzify.res.abs_2`
  Real svar_abs_2;
  /// Local, invisible local: `dy2fuzzify.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `dy2fuzzify.res.abs_0`
  Real svar_abs_0;
  /// Call to `dy2calc` ([fuzzy.lus line 261](../src/lus/fuzzy.lus.html#261)).
  Dy2calc dy2calc_5;
  /// Call to `dy2calc` ([fuzzy.lus line 262](../src/lus/fuzzy.lus.html#262)).
  Dy2calc dy2calc_4;
  /// Call to `dy2calc` ([fuzzy.lus line 263](../src/lus/fuzzy.lus.html#263)).
  Dy2calc dy2calc_3;
  /// Call to `dy2calc` ([fuzzy.lus line 264](../src/lus/fuzzy.lus.html#264)).
  Dy2calc dy2calc_2;
  /// Call to `dy2calc` ([fuzzy.lus line 265](../src/lus/fuzzy.lus.html#265)).
  Dy2calc dy2calc_1;
  /// Call to `dy2calc` ([fuzzy.lus line 266](../src/lus/fuzzy.lus.html#266)).
  Dy2calc dy2calc_0;
};

/// Stores the state for sub-node `kmlogic`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `pinput` | Real |
/// | `dinput` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `g` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `dy2fuzzify` | [Dy2fuzzify](struct.Dy2fuzzify.html) | `dlo`, `dinput` | `abs_13` | [fuzzy.lus line 101](../src/lus/fuzzy.lus.html#101) |
/// | `dy1fuzzify` | [Dy1fuzzify](struct.Dy1fuzzify.html) | `dlo`, `dinput` | `abs_12` | [fuzzy.lus line 100](../src/lus/fuzzy.lus.html#100) |
/// | `y2fuzzify` | [Y2fuzzify](struct.Y2fuzzify.html) | `plo`, `pinput` | `abs_11` | [fuzzy.lus line 97](../src/lus/fuzzy.lus.html#97) |
/// | `y1fuzzify` | [Y1fuzzify](struct.Y1fuzzify.html) | `plo`, `pinput` | `abs_10` | [fuzzy.lus line 96](../src/lus/fuzzy.lus.html#96) |
/// | `rule_base` | [Rule_base](struct.Rule_base.html) | `abs_8` | `abs_9` | [fuzzy.lus line 93](../src/lus/fuzzy.lus.html#93) |
/// | `rule_base` | [Rule_base](struct.Rule_base.html) | `abs_6` | `abs_7` | [fuzzy.lus line 92](../src/lus/fuzzy.lus.html#92) |
/// | `rule_base` | [Rule_base](struct.Rule_base.html) | `abs_4` | `abs_5` | [fuzzy.lus line 89](../src/lus/fuzzy.lus.html#89) |
/// | `rule_base` | [Rule_base](struct.Rule_base.html) | `abs_2` | `abs_3` | [fuzzy.lus line 88](../src/lus/fuzzy.lus.html#88) |
/// | `findlocation_d` | [Findlocation_d](struct.Findlocation_d.html) | `dinput` | `abs_1` | [fuzzy.lus line 73](../src/lus/fuzzy.lus.html#73) |
/// | `findlocation_p` | [Findlocation_p](struct.Findlocation_p.html) | `pinput` | `abs_0` | [fuzzy.lus line 72](../src/lus/fuzzy.lus.html#72) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct Kmlogic {
  /// Input: `kmlogic.usr.pinput`
  Real svar_pinput;
  /// Input: `kmlogic.usr.dinput`
  Real svar_dinput;
  /// Output: `kmlogic.usr.g`
  Real svar_g;
  /// Local, alias(kmlogic.impl.usr.dy2): `kmlogic.res.abs_13`
  Real svar_abs_13;
  /// Local, alias(kmlogic.impl.usr.dy1): `kmlogic.res.abs_12`
  Real svar_abs_12;
  /// Local, alias(kmlogic.impl.usr.py2): `kmlogic.res.abs_11`
  Real svar_abs_11;
  /// Local, alias(kmlogic.impl.usr.py1): `kmlogic.res.abs_10`
  Real svar_abs_10;
  /// Local, alias(kmlogic.impl.usr.second_row_2): `kmlogic.res.abs_9`
  Real svar_abs_9;
  /// Local, invisible local: `kmlogic.res.abs_8`
  Int svar_abs_8;
  /// Local, alias(kmlogic.impl.usr.second_row_1): `kmlogic.res.abs_7`
  Real svar_abs_7;
  /// Local, invisible local: `kmlogic.res.abs_6`
  Int svar_abs_6;
  /// Local, alias(kmlogic.impl.usr.first_row_2): `kmlogic.res.abs_5`
  Real svar_abs_5;
  /// Local, invisible local: `kmlogic.res.abs_4`
  Int svar_abs_4;
  /// Local, alias(kmlogic.impl.usr.first_row_1): `kmlogic.res.abs_3`
  Real svar_abs_3;
  /// Local, invisible local: `kmlogic.res.abs_2`
  Int svar_abs_2;
  /// Local, alias(kmlogic.impl.usr.dlo): `kmlogic.res.abs_1`
  Int svar_abs_1;
  /// Local, alias(kmlogic.impl.usr.plo): `kmlogic.res.abs_0`
  Int svar_abs_0;
  /// Local, local: `kmlogic.impl.usr.dindex2`
  Int svar_dindex2;
  /// Local, local: `kmlogic.impl.usr.dindex1`
  Int svar_dindex1;
  /// Local, local: `kmlogic.impl.usr.dy2`
  Real svar_dy2;
  /// Local, local: `kmlogic.impl.usr.dy1`
  Real svar_dy1;
  /// Local, local: `kmlogic.impl.usr.py2`
  Real svar_py2;
  /// Local, local: `kmlogic.impl.usr.py1`
  Real svar_py1;
  /// Local, local: `kmlogic.impl.usr.second_row_2`
  Real svar_second_row_2;
  /// Local, local: `kmlogic.impl.usr.second_row_1`
  Real svar_second_row_1;
  /// Local, local: `kmlogic.impl.usr.first_row_2`
  Real svar_first_row_2;
  /// Local, local: `kmlogic.impl.usr.first_row_1`
  Real svar_first_row_1;
  /// Local, local: `kmlogic.impl.usr.dloc`
  Int svar_dloc;
  /// Local, local: `kmlogic.impl.usr.ploc_next`
  Int svar_ploc_next;
  /// Local, local: `kmlogic.impl.usr.ploc`
  Int svar_ploc;
  /// Local, local: `kmlogic.impl.usr.dlo`
  Int svar_dlo;
  /// Local, local: `kmlogic.impl.usr.plo`
  Int svar_plo;
  /// Call to `findlocation_p` ([fuzzy.lus line 72](../src/lus/fuzzy.lus.html#72)).
  Findlocation_p findlocation_p_9;
  /// Call to `findlocation_d` ([fuzzy.lus line 73](../src/lus/fuzzy.lus.html#73)).
  Findlocation_d findlocation_d_8;
  /// Call to `rule_base` ([fuzzy.lus line 88](../src/lus/fuzzy.lus.html#88)).
  Rule_base rule_base_7;
  /// Call to `rule_base` ([fuzzy.lus line 89](../src/lus/fuzzy.lus.html#89)).
  Rule_base rule_base_6;
  /// Call to `rule_base` ([fuzzy.lus line 92](../src/lus/fuzzy.lus.html#92)).
  Rule_base rule_base_5;
  /// Call to `rule_base` ([fuzzy.lus line 93](../src/lus/fuzzy.lus.html#93)).
  Rule_base rule_base_4;
  /// Call to `y1fuzzify` ([fuzzy.lus line 96](../src/lus/fuzzy.lus.html#96)).
  Y1fuzzify y1fuzzify_3;
  /// Call to `y2fuzzify` ([fuzzy.lus line 97](../src/lus/fuzzy.lus.html#97)).
  Y2fuzzify y2fuzzify_2;
  /// Call to `dy1fuzzify` ([fuzzy.lus line 100](../src/lus/fuzzy.lus.html#100)).
  Dy1fuzzify dy1fuzzify_1;
  /// Call to `dy2fuzzify` ([fuzzy.lus line 101](../src/lus/fuzzy.lus.html#101)).
  Dy2fuzzify dy2fuzzify_0;
};

/// Stores the state for **top node** `FUZ`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Input` | Real |
/// | `Now` | Real |
/// | `Kp` | Real |
/// | `Ki` | Real |
/// | `Kd` | Real |
/// | `Setpoint` | Real |
/// | `SampleTime` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Output` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `kmlogic` | [Kmlogic](struct.Kmlogic.html) | `error`, `dInput` | `abs_2` | [fuzzy.lus line 15](../src/lus/fuzzy.lus.html#15) |
/// | `limit` | [Limit](struct.Limit.html) | `abs_0` | `abs_1` | [fuzzy.lus line 10](../src/lus/fuzzy.lus.html#10) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct FUZ {
  /// Input: `FUZ.usr.Input`
  Real svar_Input;
  /// Input: `FUZ.usr.Now`
  Real svar_Now;
  /// Input: `FUZ.usr.Kp`
  Real svar_Kp;
  /// Input: `FUZ.usr.Ki`
  Real svar_Ki;
  /// Input: `FUZ.usr.Kd`
  Real svar_Kd;
  /// Input: `FUZ.usr.Setpoint`
  Real svar_Setpoint;
  /// Input: `FUZ.usr.SampleTime`
  Real svar_SampleTime;
  /// Output: `FUZ.usr.Output`
  Real svar_Output;
  /// Local, call: `FUZ.res.abs_2`
  Real svar_abs_2;
  /// Local, alias(FUZ.impl.usr.outputSum): `FUZ.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `FUZ.res.abs_0`
  Real svar_abs_0;
  /// Local, local: `FUZ.impl.usr.outputSum`
  Real svar_outputSum;
  /// Local, local: `FUZ.impl.usr.error`
  Real svar_error;
  /// Local, local: `FUZ.impl.usr.dInput`
  Real svar_dInput;
  /// Call to `limit` ([fuzzy.lus line 10](../src/lus/fuzzy.lus.html#10)).
  Limit limit_1;
  /// Call to `kmlogic` ([fuzzy.lus line 15](../src/lus/fuzzy.lus.html#15)).
  Kmlogic kmlogic_0;
};

/// Stores the state for sub-node `PID_calc`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Input` | Real |
/// | `Now` | Real |
/// | `Setpoint` | Real |
/// | `Kp` | Real |
/// | `Ki` | Real |
/// | `Kd` | Real |
/// | `SampleTime` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Output` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `limit` | [Limit](struct.Limit.html) | `abs_2` | `abs_3` | [pid2.lus line 36](../src/lus/pid2.lus.html#36) |
/// | `limit` | [Limit](struct.Limit.html) | `abs_0` | `abs_1` | [pid2.lus line 34](../src/lus/pid2.lus.html#34) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct PID_calc {
  /// Input: `PID_calc.usr.Input`
  Real svar_Input;
  /// Input: `PID_calc.usr.Now`
  Real svar_Now;
  /// Input: `PID_calc.usr.Setpoint`
  Real svar_Setpoint;
  /// Input: `PID_calc.usr.Kp`
  Real svar_Kp;
  /// Input: `PID_calc.usr.Ki`
  Real svar_Ki;
  /// Input: `PID_calc.usr.Kd`
  Real svar_Kd;
  /// Input: `PID_calc.usr.SampleTime`
  Real svar_SampleTime;
  /// Output: `PID_calc.usr.Output`
  Real svar_Output;
  /// Local, alias(PID_calc.usr.Output): `PID_calc.res.abs_3`
  Real svar_abs_3;
  /// Local, invisible local: `PID_calc.res.abs_2`
  Real svar_abs_2;
  /// Local, alias(PID_calc.impl.usr.outputSum): `PID_calc.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `PID_calc.res.abs_0`
  Real svar_abs_0;
  /// Local, local: `PID_calc.impl.usr.outputSum`
  Real svar_outputSum;
  /// Local, local: `PID_calc.impl.usr.error`
  Real svar_error;
  /// Local, local: `PID_calc.impl.usr.dInput`
  Real svar_dInput;
  /// Call to `limit` ([pid2.lus line 34](../src/lus/pid2.lus.html#34)).
  Limit limit_1;
  /// Call to `limit` ([pid2.lus line 36](../src/lus/pid2.lus.html#36)).
  Limit limit_0;
};

/// Stores the state for **top node** `PID`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Input` | Real |
/// | `Now` | Real |
/// | `Setpoint` | Real |
/// | `Kp` | Real |
/// | `Ki` | Real |
/// | `Kd` | Real |
/// | `SampleTime` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Output` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `PID_calc` | [PID_calc](struct.PID_calc.html) | `Input`, `Now`, `Setpoint`, `Kp`, `Ki`, `Kd`, `SampleTime` | `abs_0` | [pid2.lus line 13](../src/lus/pid2.lus.html#13) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
struct PID {
  /// Input: `PID.usr.Input`
  Real svar_Input;
  /// Input: `PID.usr.Now`
  Real svar_Now;
  /// Input: `PID.usr.Setpoint`
  Real svar_Setpoint;
  /// Input: `PID.usr.Kp`
  Real svar_Kp;
  /// Input: `PID.usr.Ki`
  Real svar_Ki;
  /// Input: `PID.usr.Kd`
  Real svar_Kd;
  /// Input: `PID.usr.SampleTime`
  Real svar_SampleTime;
  /// Output: `PID.usr.Output`
  Real svar_Output;
  /// Local, call: `PID.res.abs_0`
  Real svar_abs_0;
  /// Call to `PID_calc` ([pid2.lus line 13](../src/lus/pid2.lus.html#13)).
  PID_calc PID_calc_0;
};

struct PIDC {
  double kp;
  double ki;
  double kd;
  double SampleTime;
  double Output;
  double Setpoint;
  double errSum;
  double lastInput;
  double lastTime;
  double outputSum;
};

extern "C" {

double ComputeLustreFUZ(FUZ *raw_ptr,
                        double input,
                        double now,
                        double setpoint,
                        double Kp,
                        double Ki,
                        double Kd,
                        double sampleTime);

double ComputeLustrePID(PID *raw_ptr,
                        double input,
                        double now,
                        double setpoint,
                        double Kp,
                        double Ki,
                        double Kd,
                        double sampleTime);

void InitLustreFUZ(FUZ *raw_ptr,
                   double input,
                   double now,
                   double setpoint,
                   double Kp,
                   double Ki,
                   double Kd,
                   double sampleTime);

void InitLustrePID(PID *raw_ptr,
                   double input,
                   double now,
                   double setpoint,
                   double Kp,
                   double Ki,
                   double Kd,
                   double sampleTime);

double compute_PIDC(PIDC *raw_ptr, double Input, double now);

void compute_pid(double Input,
                 double *Output,
                 double Setpoint,
                 double Kp,
                 double Ki,
                 double Kd,
                 double now,
                 double *lastTime,
                 double SampleTime,
                 double *lastInput,
                 double *outputSum);

void create_PIDC(PIDC *raw_ptr, double kp, double ki, double kd, double SampleTime);

void drop_PIDC(PIDC *x);

double wraptopi_r(double x);

} // extern "C"
