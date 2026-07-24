# Edge Impulse model for Zephyr
Model package name: Tutorial_Responding_to_your_voic
Model version: 115
Edge Impulse SDK version: v1.93.27

## Instructions
To add the model:
Copy model-parameters, tflite-model and zephyr folders and CMakeList.txt inside your project.
Then, add the following line to your CMakeList.txt (need to be done just once!):
list(APPEND ZEPHYR_EXTRA_MODULES /model_folder)
where model_folder is the path to where you copied the folders extracted.

To add the Edge Impulse SDK to your project, copy the the entry of edge-impulse-sdk-zephyr from the extracted `west.yml` into your manifest (of zephyr or your project, depends on the topology in use) starting from the line to reference the Edge Impulse SDK version needed for this model.
You need to copy:
```
    - name: edge-impulse-sdk-zephyr
      description: Edge Impulse SDK for Zephyr
      path: modules/edge-impulse-sdk-zephyr
      west-commands: west/west-commands.yml
      revision: v1.93.27
      url: https://github.com/edgeimpulse/edge-impulse-sdk-zephyr
```

Check if the revision of the `edge-impulse-sdk-zephyr` module in your manifest file matches with the one needed for this model which is v1.93.27.
If is not the case, or is it the first time you integrate the Edge Impulse SDK, update your manifest file with the above entry and then call `west update` to pull the required version.

## Additional west commands
The Edge Impulse SDK for Zephyr provides additional west commands to ease the building and deployment of Edge Impulse models.
Two commands are provided:
- `west ei-build`: to build the model for a specific board
- `west ei-deploy`: to deploy the built model to a connected device

To see the parameters for these commands, run:
```
west ei-build --help
west ei-deploy --help
```

For example, if you want to build and deploy the Tutorial: Continuous motion recognition https://studio.edgeimpulse.com/studio/14299, you can use the following commands:
```
west ei-build -p 14299 -k ei_26de8eccc6a4aded83c7a4d7d967d22c7cae2de81a537edbdf15bd814753e8dc
west ei-deploy -p 14299 -k ei_26de8eccc6a4aded83c7a4d7d967d22c7cae2de81a537edbdf15bd814753e8dc
```

Happy inferencing!

