[![Discord](https://img.shields.io/discord/1028718524182577283?label=Discord%20)](https://discord.gg/GGthsd4NRy)
# EngineSimulatorPlugin

This plugin integrates EngineSimulator into the Chaos Vehicle simulation. The plugin provides a vehicle movement component that subclasses `UChaosWheeledVehicleMovementComponent`, and as such it is a drop-in replacement for Unreal's vehicle movement.

# Usage

Currently this plugin only supports Unreal Engine 5.1+

This guide also assumes that your project is using Chaos vehicles or the Vehicle Template.

1. Download the plugin from the Releases page, put it into your project's plugin folder and enable it inside your project's plugin settings.
2. Open your vehicle blueprint. Select your Chaos Vehicle Movement Component, and change the Component Class from `ChaosWheeledVehicleMovementComponent` to `EngineSimulatorWheeledVehicleMovementComponent`
![ReplaceComponentClass](https://user-images.githubusercontent.com/8600772/226207696-951aaae3-2547-40ae-81bf-407d0fb5bed6.png)
3. Add in a `EngineSimulatorAudio` component.
4. Add in a Gear-Up and Gear-Down input binding, and call SetEngineSimChangeGearUp and SetEngineSimChangeGearDown
![gear_change](https://user-images.githubusercontent.com/8600772/226207860-61120a57-6a01-4a57-bf23-aa2049ff0b5c.png)

To change the engine, edit [main.mr](Resources/assets/main.mr). All of the Engine Simulator config files are in the Resources folder.

That is all you need to do to get started with this plugin. Extra data (such as RPM, HP, number of gears, engine name, etc) can be accessed through the `LastEngineSimulatorOutput` variable on the movement component.

## Changing engine sounds in-game
You can change the Engine Sounds by overwriting the `main.mr` file. For illustration purpose I will show an example how to do this. 

I use the [File Helper Blueprint Library](https://www.unrealengine.com/marketplace/en-US/product/file-helper-bp-library) from Unreal Marketplace. 

![image](https://github.com/1NoobDev/EngineSimulatorPlugin/assets/20015201/ba59d3a4-2c43-4f80-885c-f00d20a8cde8)

The `File` is a local variable with the path to the main.mr file. It's also working when the project is build, just make sure to adjust the path of main.mr from plugin/resources towards the build path. 
