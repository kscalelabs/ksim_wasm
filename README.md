<p align="center">
  <a href="https://zalo.github.io/mujoco_wasm/"><img src="./examples/MuJoCoWasmLogo.png" href></a>
</p>

<div align="center">

[![License](https://img.shields.io/badge/license-MIT-green)](https://github.com/kscalelabs/ksim/blob/main/LICENSE)
[![Discord](https://img.shields.io/discord/1224056091017478166)](https://discord.gg/k5mSvCkYQh)
[![Wiki](https://img.shields.io/badge/wiki-humanoids-black)](https://humanoids.wiki)

</div>

## The Power of MuJoCo in your Browser.

Load and Run MuJoCo 2.3.1 Models using JavaScript and WebAssembly.

This repo is a fork of @zalo 's starter repository, adding tons of functionality and stompy robot scene from kscale labs. 


### [See the Live Demo Here](https://zalo.github.io/mujoco_wasm/)

### [See a more Advanced Example Here](https://kzakka.com/robopianist/)

## Building

### [See the wiki page Here](https://humanoids.wiki/w/MuJoCo_WASM)

## JavaScript API

```javascript
import load_mujoco from "./mujoco_wasm.js";

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/humanoid.xml", await (await fetch("./examples/scenes/humanoid.xml")).text());

// Load in the state from XML
let model       = new mujoco.Model("/working/humanoid.xml");
let state       = new mujoco.State(model);
let simulation  = new mujoco.Simulation(model, state);
```

Typescript definitions are available.

## Work In Progress Disclaimer

So far, most mjModel and mjData state variables and functions (that do not require custom structs) are exposed.

At some point, I'd like to de-opinionate the binding and make it match the original MuJoCo API better.
