/**
 * =============================================================================
 * renderer.ts
 * =============================================================================
 * 
 * Source code of the 3D visualization developed as a part of the
 * "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
 * 
 * This file has been partially initialized by npm init electron-app@latest
 * 
 * Author: Jakub Výmola (kuba.vymola@gmail.com)
 * Date: 04/30/2023
 * 
 * =============================================================================
 * 
 * This file will automatically be loaded by webpack and run in the "renderer" context.
 * To learn more about the differences between the "main" and the "renderer" context in
 * Electron, visit:
 *
 * https://electronjs.org/docs/latest/tutorial/process-model
 *
 * By default, Node.js integration in this file is disabled. When enabling Node.js integration
 * in a renderer process, please be aware of potential security implications. You can read
 * more about security risks here:
 *
 * https://electronjs.org/docs/tutorial/security
 *
 * To enable Node.js integration in this file, open up `main.js` and enable the `nodeIntegration`
 * flag:
 *
 * ```
 *  // Create the browser window.
 *  mainWindow = new BrowserWindow({
 *    width: 800,
 *    height: 600,
 *    webPreferences: {
 *      nodeIntegration: true
 *    }
 *  });
 * ```
 */

import {
    Engine,
    Scene,
} from '@babylonjs/core';

import App from './app/app';

export default class Renderer {
    private _scene: Scene;
    private _app: App;
    
    createApp(canvas: HTMLCanvasElement, engine: Engine, scene: Scene) {
        this._app = new App(canvas, engine, scene);
    }

    initialize(canvas: HTMLCanvasElement) {
        const engine = new Engine(canvas, true);
        
        const scene = new Scene(engine);
        this._scene = scene;

        this.createApp(canvas, engine, scene);

        engine.runRenderLoop(() => {
            this._scene.render();
        })

        window.addEventListener('resize', function () {
            engine.resize();
        });
    }
}

const renderer: Renderer = new Renderer();
renderer.initialize(document.querySelector('#render-canvas') as HTMLCanvasElement);
