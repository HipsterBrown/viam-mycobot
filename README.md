# `mycobot` module

This [module](https://docs.viam.com/registry/modular-resources/) implements the [`rdk:component:arm` API](https://docs.viam.com/appendix/apis/components/arm/) and [`rdk:component:gripper` API](https://docs.viam.com/appendix/apis/components/gripper/) for the [Elephant Robotics myCobot 280 Pi](https://www.elephantrobotics.com/en/mycobot-280-pi-2023-en/).

With this model, you can control a desktop robotic arm that can be used for testing before deploying to an industrial arm, home or office collaboration tasks, or creating a face tracking camera arm.

## Requirements

This module assumes you are using the myCobot 280 Raspberry Pi edition with the latest version of Ubuntu [provided by Elephant Robotics](https://www.elephantrobotics.com/en/downloads/) and the latest version of the [AtomMain firmware](https://docs.elephantrobotics.com/docs/mycobot_280_pi_en/3-FunctionsAndApplications/5.BasicFunction/5.2-Softwarelnstructions/) flashed to the internal ESP32 microcontroller

## Configure your mycobot280 arm component

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in the [Viam app](https://app.viam.com/).
[Add `hipsterbrown:mycobot` to your machine](https://docs.viam.com/configure/#components).

### Attributes

The following attributes are available for `hipsterbrown:arm:mycobot280` arm component:

| Name    | Type   | Required?    | Default | Description |
| ------- | ------ | ------------ | ------- | ----------- |
| `default_speed` | number (1-100) | Optional | 20  | The default speed to use when moving the joints on the arm set as millimeters per second |


### Example configuration

```json
{
    "default_speed": 50
}
```

## Configure your mycobot gripper component

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in the [Viam app](https://app.viam.com/).
[Add `hipsterbrown:mycobot` to your machine](https://docs.viam.com/configure/#components).

### Attributes

The following attributes are available for `hipsterbrown:gripper:mycobot` arm component:

| Name    | Type   | Required?    | Default | Description |
| ------- | ------ | ------------ | ------- | ----------- |
| `arm_name` | string | Required | N/A  | The name of the myCobot280 arm component used with gripper |
| `default_speed` | number (1-100) | Optional | 20  | The default speed to use when moving the joints on the arm set as millimeters per second |


### Example configuration

```json
{
    "arm_name": "arm-1",
    "default_speed": 30
}
```

### Next steps

Start automating everyday tasks with your desktop robotic arm, like [dipping apples in honey](https://codelabs.viam.com/guide/shana-tobot/index.html?index=..%2F..index#3)!

## Troubleshooting

After power cycling (turning the arm off and on using the power switch), the Atom firmware may need to be reflashed to get the arm working again. [See this GitHub issue for more information.](https://github.com/elephantrobotics/myCobot/issues/48)
