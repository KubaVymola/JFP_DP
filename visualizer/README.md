# Instalation

Run:

```
$ rm package-lock.json  # not removing causes trouble with babylon.js dependencies
$ npm install
```

It might be neccessary to manually run, but do not try this, unless you have verified, that
`babylon.js` is really missing those dependencies:

```
$ npm install @babylonjs/materials
$ npm install @babylonjs/gui
$ npm install @babylonjs/gui-editor
$ npm install @babylonjs/serializers
$ npm install @types/react
$ npm install @types/react-dom
$ npm install babylonjs-gltf2interface
```

# Running

```
$ npm start
```
