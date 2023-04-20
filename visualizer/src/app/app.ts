//==============================================================================
// app.ts
//==============================================================================
//
// Source code of the 3D visualization developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

/* eslint-disable @typescript-eslint/no-explicit-any */
import "@babylonjs/core/Debug/debugLayer";
import "@babylonjs/inspector";
import "@babylonjs/loaders/glTF";
import {
    Engine,
    Scene,
    Vector3,
    HemisphericLight,
    Mesh,
    StandardMaterial,
    AxesViewer,
    CreateTiledPlane,
    Texture,
} from "@babylonjs/core";
import "@babylonjs/loaders";

import OriginCamera from "./originCamera";
import Entity from "./entity";
import Craft from "./craft";
import {
    createCamera,
    createEarth,
    createEntity,
} from './utils'
import {
    EARTH_RADIUS,
    SIM_DATA_ORIGIN,
} from './constants';

class App {
    private _ws: WebSocket;
    private _wsConnectTimeoutId: NodeJS.Timeout = null;
    private _connectButton: HTMLButtonElement;
    private _urlInput: HTMLInputElement;
    
    private _camera: OriginCamera;
    private _craft: Craft;

    private _engine: Engine;
    private _scene: Scene;

    pressedKeys: any = {};
    
    constructor(canvas: HTMLCanvasElement, engine: Engine, scene: Scene) {
        document.addEventListener('keydown', (e: KeyboardEvent) => {
            this.pressedKeys[e.key] = true;
        });
        document.addEventListener('keyup', (e: KeyboardEvent) => {
            this.pressedKeys[e.key] = false;
        });

        /**
         * initialize babylon scene and engine
         */
        this._engine = engine;
        this._scene = scene;
        
        this._scene.clearColor.set(0, 0, 0, 1);
        this._scene.useRightHandedSystem = true;


        this._camera = createCamera(this._scene, canvas);
        this._camera.upVector = new Vector3(0, 0, 1);

        /**
         * Create Earth
         */
        const earthEntity: Entity = createEntity('earthEntity', this._scene, this._camera);
        const earthSphere: Mesh = createEarth(this._scene, earthEntity);

        const light: HemisphericLight = new HemisphericLight("light1", new Vector3(1, 0, 0), this._scene);

        /**
         * Create origin axes
         */
        const axes = new AxesViewer(this._scene, 0.5);
        const axesEntity = createEntity('axesEntity', this._scene, this._camera);
        axesEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        axes.xAxis.parent = axesEntity;
        axes.yAxis.parent = axesEntity;
        axes.zAxis.parent = axesEntity;

        

        /**
         * Create basic ground
         */
         const options = {
            sideOrientation: Mesh.DOUBLESIDE,
            pattern: Mesh.NO_FLIP,
            width: 50,
            height: 50,
            tileSize: 1,
            tileWidth:1
        }
        
        const tiledMaterial = new StandardMaterial("");
        tiledMaterial.diffuseTexture = new Texture("../assets/tiled_grass.jpg");
        
        const groundPlane = CreateTiledPlane('groundPlane', options);
        groundPlane.position = new Vector3(0, 0, 0);
        groundPlane.rotation = new Vector3(Math.PI, 0, 0);
        const groundPlaneEntity = createEntity('groundPlaneEntity', this._scene, this._camera);
        groundPlaneEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        groundPlane.parent = groundPlaneEntity;
        groundPlane.material = tiledMaterial;
        
        /**
         * Hide/show the Inspector
         */
        window.addEventListener("keydown", (ev) => {
            if (ev.shiftKey && ev.ctrlKey && ev.key === 'I') {
                if (this._scene.debugLayer.isVisible()) {
                    this._scene.debugLayer.hide();
                } else {
                    this._scene.debugLayer.show();
                }
            }
        });

        /**
         * Connect to websocket
         */
        this._urlInput = document.querySelector('#connect-input');
        this._urlInput.value = SIM_DATA_ORIGIN;
        
        this._connectButton = document.querySelector('#connect-button');
        this._connectButton.onclick = () => {
            this.connect(false);
        }

        /**
         * Create craft
         */
        this._craft = new Craft(this._scene, this._camera);

        /**
         * Start running
         */
        canvas.focus();
    }

    connect(reconnect: boolean) {
        if (reconnect || !this._ws) {
            this._ws = new WebSocket(this._urlInput.value);

            this._connectButton.innerText = 'Connecting...';
            
            this._ws.onopen = () => {
                console.log('Connected');

                this._connectButton.innerText = 'Disconnect';

                this._ws.onmessage = (event) => {
                    const parsedData = JSON.parse(event.data);

                    // console.log(parsedData);
                    this._craft.update(parsedData, this._camera);

                    const data = this._craft.getGamepadData();
                    this._ws.send(JSON.stringify(data));
                };
            };
            
            this._ws.onclose = () => {
                this._wsConnectTimeoutId = setTimeout(() => {
                    console.log('Close: Trying to reconnect');
                    this.connect(true);
                }, 250);
            }

            
        } else {
            if (this._wsConnectTimeoutId) clearTimeout(this._wsConnectTimeoutId);
            this._ws.onclose = null;
            this._ws.close();
            this._ws = null;

            this._connectButton.innerText = 'Connect';
        }
    }
}

export default App;
