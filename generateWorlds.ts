import * as fs from "jsr:@std/fs";

const worldInfos: ["trainDebug" ,"train" , "animator" , "walker" , "inference"] = ["trainDebug", "train", "animator", "walker", "inference"]

const folderPath = import.meta.dirname + "/testWorlds"

console.log(folderPath)

if (fs.existsSync(folderPath)) {
  const resp = prompt(
    "Are you sure you want to continue? [y/N]\nA worlds folder already exists, this will delete it if you continue",
  )?.toLowerCase();

  if (!(resp == "y" || resp == "yes")) {
    Deno.exit();
  }
}



try {
  Deno.removeSync(folderPath, {recursive: true})
} catch (_error) {/**/}

fs.ensureDirSync(folderPath)

for (const worldInfo of worldInfos) {
    const content = `
    #VRML_SIM R2025a utf8

    EXTERNPROTO "../protos/noonRobot.proto"
    ${worldInfo == "train" ? "" : `
      EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
      EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
      EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
      
      Viewpoint {
        orientation 0.12787361201113046 0.14181193443643453 -0.9815995693777699 1.50360476832192
        position -0.5820072762877054 8.475077378833792 1.959752940496758
      }
      TexturedBackgroundLight {
      }
      TexturedBackground {
      }
        
    `}

    Floor {
      contactMaterial "floor"
      size 2000 2000
      
      ${worldInfo == "train" ? "appearance NULL" : "" }
        
    }

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

    ${()=>{
      switch (worldInfo) {
        case "animator":
          
          return `noonRobot {
              controller "animator"
              cylinderSubdivision 16
            }`
      
        case "inference":
          return `noonRobot {
            controller "<extern>"
            inference TRUE
          }`
          

        case "trainDebug":
          return `noonRobot {
              controller "<extern>"
              name "trainer"
            }
            DEF TRAINER noonRobotSpawner {
              count 0
              spacing 30
            }`
        case "train":
          return `noonRobot {
            controller "<extern>"
            name "trainer"
          }
          DEF TRAINER noonRobotSpawner {
            count 0
            spacing 100
          }`
        case "walker":
          return `noonRobot {
            controller "walker"
            cylinderSubdivision 16
          }`
      }
    }}
    `

    Deno.writeFile(`${folderPath}/${worldInfo}.wbt`, new TextEncoder().encode(content))
}