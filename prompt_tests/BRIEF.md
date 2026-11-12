# Writing a script from its prompt alone

You are the model a reader of *Mechanical Design in Autodesk Fusion with Claude* gives a printed prompt to. Your job
is to write the Fusion Python script that prompt asks for, so that we can learn whether the prompt is enough to get a
script that runs and builds the right part or assembly.

## The prompt

`{PROMPT}` (inside the companion repository at `C:\Users\eamer\Documents\MechDesignFusionClaude\companion`).

## What you may read

- The prompt.
- Every file the prompt names by its place (libraries under `.../lib/`, other scripts it tells you to load and call,
  results it tells you to read). Load them as the prompt says (loaded, not copied).
- `common/companion_paths.py` and `common/lib/mdfc_parts.py` (how a script finds the libraries, and the parts library).
- `FUSION_API_TRAPS.md` at the companion's root: what Fusion's API does that its documentation does not say. A reader
  gives this file to the model with every prompt; read it before you write.
- `tools/fusion_check.py` and the check `{CHECK}`, ONLY to learn how the check drives a script: the functions and
  module-level names it calls or reads (for example `build()`, `run()`, `_Created`, `VERSION`, `_last`, `set_state()`),
  the ids of the dialog's inputs it types into, and the names of components and joints it looks up. Do not take any
  design fact (a size, a position, a rule, a number) from the check. If the check needs a design fact the prompt does
  not give, do not use it: write it down under "Prompt gaps" in your report.

## What you must not read

- The script this prompt belongs to: anything in `{SCRIPT_DIR}`.
- The project's `controls/` and `captures/`, and anything else under `prompt_tests/` except this brief.
- Any other script of the companion that the prompt does not name.

## What you write

- `prompt_tests/{NAME}/{NAME}.py` and `prompt_tests/{NAME}/{NAME}.manifest`, nothing else.
- The script finds the companion's libraries the way the prompt says, through `common/companion_paths.py` (walk up from
  the script's folder to the first one that holds `common/companion_paths.py`, put `common/` on `sys.path`, import it).
- Check it compiles: `python -m py_compile prompt_tests/{NAME}/{NAME}.py`.
- Do not run Fusion and do not call tools/fusion_mcp.py or tools/run_check.py: Fusion is busy, and the run is done for
  you afterwards.

## Your report (your last message)

1. What you built, in two or three sentences.
2. **Prompt gaps**: every place where the prompt left something to guess, was ambiguous, or contradicted a file it
   names, with the line of the prompt and what you chose. Say "none" if there were none.
3. **Interface taken from the check**: the names and ids you took from the check to let it drive the script.
4. Anything you could not do.
