const worldInfos: ["trainDebug" ,"train" , "animator" , "walker" , "inference"] = ["trainDebug", "train", "animator", "walker", "inference"]

const worldTexts: { name: string, content: string }[] = []



for (const worldInfo of worldInfos) {
    let text = `
    #VRML_SIM R2025a utf8

    EXTERNPROTO "../protos/noonRobot.proto"
    ${worldInfo == "train" ? "" : `
        EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
        EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
        EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"    
        
    `}

    ${worldInfo == "trainDebug" || "train" ? `EXTERNPROTO "../protos/noonRobotSpawner.proto"`:""}

    DEF WorldInfo WorldInfo {
        ERP 0.3
        basicTimeStep 16
        physicsDisableLinearThreshold 0.005
        physicsDisableAngularThreshold 0.005
        contactProperties [
          ContactProperties {
            material2 "floor"
            coulombFriction [
              1.2, 1
            ]
            rollingFriction 0.1 0.1 0.1
            bounce 0
            bounceVelocity 0
            forceDependentSlip [
              0.001
            ]
            softERP 0.5
          }
        ]
    }


    `
}