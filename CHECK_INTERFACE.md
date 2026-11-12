# What a check needs from a script

Each project's check runs a script the way a reader does: it runs it, waits for its dialog, types into the dialog,
presses OK, and then measures what Fusion built. To do that it relies on a few conventions that every script in
this repository follows. A script written by a model from one of the books' prompts passes its check only if it
follows them too.

The last line of each prompt's FUSION CONVENTIONS names its check and the names that check looks for. This file is
what lies behind that line. `python tools/check_names.py <Name>` prints the line again from the check itself, and
`python tools/check_names.py --all` tells you whether every prompt still carries its check's names.

## Give the check to the model, for its names only

Give the model three things: the prompt, `FUSION_API_TRAPS.md`, and the check the prompt names. Tell it to take
from the check only the names the check calls and reads, and to take every size, rule and number from the prompt.
The check works the design out again in its own code so that it can test the script. A model that copies those
numbers from it has proved nothing about the prompt.

## The conventions

1. **A version tag.** `VERSION` at module level, a string such as `'RearAxleCarrier 1.5 (2026-11-12)'`, and the same
   text in the message the script shows when it has built. The check passes only if one of the script's messages
   contains it.
2. **build().** A module-level `build(...)` that builds everything in a new design document and returns the
   message, a string. Its keyword arguments are the prompt's INPUTS, each with the prompt's default and in the
   prompt's units (mm, deg, kg, as listed), named as the prompt's last line gives them. The check calls `build()`
   itself with other values, so `build()` must not depend on the dialog.
3. **Refusals.** When the prompt says to refuse, `build()` raises `ValueError` before it opens a document, with a
   message that says why and contains the words the prompt's last line lists (a number in those words is the
   number for the refused case in ACCEPTANCE CRITERIA). Any other error is a failure, not a refusal.
4. **The dialog.** `run(context)` creates a command and executes it. The command's created handler is a
   module-level class named `_Created`, a subclass of `adsk.core.CommandCreatedEventHandler`, which adds the
   inputs with the ids the prompt's last line gives (they are `build()`'s argument names unless the line says
   otherwise) and connects an execute handler. That handler reads the inputs, converts lengths from the API's
   centimetres to millimetres, calls `build()`, and shows the message with `ui.messageBox(...)`; a `ValueError`
   is shown the same way. Keep the handlers in a module-level list so that Python does not collect them.
5. **Found by name at run time.** `run()` looks up `_Created` in the module when it runs, as plain code does. The
   check puts a subclass of its own in its place to see the command open, and replaces the text
   `ui.messageBox(` in the source to read the messages. Write the call exactly so.
6. **Nothing at import but definitions.** The check loads the file's source and calls these names itself, so
   importing the file must not open documents or build anything.
7. **What the check looks inside.** A prompt's last line may name more module-level things: `_last`, a dict that
   `build()` fills with what it made (for example `'groups'`, the top-level occurrences by name, or `'joints'`,
   the joints by name); functions such as `design(...)` (the numbers, worked without Fusion), `parts_list(root)`
   (a dict of each kind of component, named as in the prompt's parts list, and how many there are),
   `set_state(...)`, `pose(...)` or `set_pose(...)` (move the groups that were built); constants; and libraries
   imported under a short name (`axle_design imported as ad` means `import axle_design as ad`). How the check uses
   each, its arguments and the keys it reads, is in the check: that is why the model reads it.
8. **Names as given.** Components and joints are named exactly as the prompt gives them: the parts list by
   component name and count, and the joints in the last line. A name written `'Bolt <n>'` is one per bolt, numbered
   from 1; `'Half shaft <side> turns'` is one per side, with the side as the prompt names it.
