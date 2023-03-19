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

That is all you need to do to get started with this plugin.
