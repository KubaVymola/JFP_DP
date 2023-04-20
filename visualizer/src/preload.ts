//==============================================================================
// preload.ts
//==============================================================================
//
// Source code of the 3D visualization developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

// See the Electron documentation for details on how to use preload scripts:
// https://www.electronjs.org/docs/latest/tutorial/process-model#preload-scripts

import { contextBridge, ipcRenderer } from "electron";

contextBridge.exposeInMainWorld('api', {
    openDialog: (options: any) => ipcRenderer.invoke('dialog:openFile', options),
    readFile: (filePath: string, options: any) => ipcRenderer.invoke('file:read', filePath, options),
    joinPath: (...args: any) => ipcRenderer.invoke('path:join', ...args),
});