import {
    SceneLoader,
    Vector3,
    Mesh,
    Matrix,
    Quaternion,
} from '@babylonjs/core';

import Entity from './entity';
import {
    createEntity,
} from './utils';
import {
    EARTH_RADIUS,
    DEG_TO_RAD,
} from './constants';
import OriginCamera from './originCamera';

class Craft {
    craftEntity: Entity = null;
    
    constructor(scene, camera) {
        this.craftEntity = createEntity('craftEntity', scene, camera);
        this.importMeshCallback = this.importMeshCallback.bind(this);
        
        SceneLoader.ImportMesh(['sabre_light.001'], '../assets/', 'sabre_light.obj', scene, this.importMeshCallback);
        this.craftEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        this.craftEntity.setPivotPoint(new Vector3(0.3, 0, 0));
    }

    importMeshCallback(newMeshes: Mesh[]) {
        console.log('Mesh loaded');
        console.log(this.craftEntity);
            
        newMeshes.forEach((mesh: Mesh) => {
            mesh.parent = this.craftEntity;
            console.log(mesh.name);
        });
    }

    update(craftData, camera: OriginCamera) {
        /**
         * Update crafts position and rotation
         * [X] = Y   // East
         * [Y] = Z   // North
         * [Z] = X   // Up
         * s = latitude
         * t = longitude
         */

        // latitude -> North-south
        // longitude -> East-west
        const latitude_rad = craftData['latitude_rad'];
        const longitude_rad = craftData['longitude_rad'];

        const craftRadius = EARTH_RADIUS + craftData['altitude_m'];
        
        /**
         * TODO make sure that the CG is translating correctly
         */
        const craftCG = new Vector3(
            craftData['cg_x_m'],
            craftData['cg_y_m'],
            craftData['cg_z_m'],
        );
        
        const rocketPos = new Vector3(
            craftRadius * Math.sin(longitude_rad) * Math.cos(latitude_rad),   // east
            craftRadius * Math.sin(latitude_rad),                             // north
            craftRadius * Math.cos(longitude_rad) * Math.cos(latitude_rad),   // up
        )
        
        this.craftEntity.doublepos = rocketPos.subtract(craftCG);
        this.craftEntity.setPivotPoint(craftCG);
        camera.doubletgt = rocketPos;

        camera.upVector = rocketPos.normalize();
        
        /**
         * JSBSIM uses (w, i, j, k), babylon.js uses (i, j, k, w),
         * where w is the real part of quaternion. Hence the order of (2, 3, 4, 1)
         */
        const rotationQuaternion: Quaternion = new Quaternion(
            craftData['ecef_q_2'],
            craftData['ecef_q_3'],
            craftData['ecef_q_4'],
            craftData['ecef_q_1'],
        );

        let rotationMatrix: Matrix = Matrix.Identity();

        const quaternionMatrix: Matrix = Matrix.Zero()
        rotationQuaternion.toRotationMatrix(quaternionMatrix);
        
        const fromJSBSimCoords: Matrix = Matrix.Zero();
        fromJSBSimCoords.setRowFromFloats(0, 0, 0, 1, 0);
        fromJSBSimCoords.setRowFromFloats(1, 1, 0, 0, 0);
        fromJSBSimCoords.setRowFromFloats(2, 0, 1, 0, 0);
        fromJSBSimCoords.setRowFromFloats(3, 0, 0, 0, 1);

        /**
         * ! The aircraft, after loading, should have the nose to +x, bottom to +z and right +y
         */
        
        /**
         * JSBSim rotates the craft -90° around y-axis in 0-yaw 0-pich state
         * Therefore it must be ensured that once this rotation is applied,
         * the craft will be pointing to the north with the long and cross axes
         * parallel to the ground.
         * The following two matrix multiplications ensure that the craft is pointing
         * to the north.
         */
        rotationMatrix = rotationMatrix.multiply(Matrix.RotationAxis(new Vector3(0, 1, 0), Math.PI));
        rotationMatrix = rotationMatrix.multiply(Matrix.RotationAxis(new Vector3(1, 0, 0), -Math.PI/2));
        /**
         * This rotation matrix is obtained from a quaternion directly from JSBSim's
         * ECEF frame. It is right handed Up-East-North frame
         */
        rotationMatrix = rotationMatrix.multiply(quaternionMatrix);
        /**
         * Transform the UEN frame to a frame that I use, which is ENU
         */
        rotationMatrix = rotationMatrix.multiply(fromJSBSimCoords);
        
        this.craftEntity.rotationQuaternion = Quaternion.FromRotationMatrix(rotationMatrix);
    }


}

export default Craft;
