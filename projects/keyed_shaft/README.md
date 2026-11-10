# projects/keyed_shaft

Volume 1, Chapter 6 (Shafts, Keys and Hubs).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `KeyedShaft/`: Chapter 6: a shaft turning in a bearing block, with a hub fixed to it by a DIN 6885 parallel key; the key's stresses checked against the torque.

## checks/ (1): run the script as a user does and measure what Fusion built

- `keyed_shaft_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `keyed_shaft_key_on_top`: breaks "The key sits with t1 of its height in the shaft's slot and the rest in the hub's slot"

## captures/ (1): the pictures in the book

`keyed_shaft`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
