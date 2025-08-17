I want to work on a automatic data_engine that usees viual prompts to interact with the ai models. What I want is to create a gui application build in python with the library pyside6. This gui should take the user inputs and ingest it t an ai model to at the end generate a yolo training dataset. So we will be using the model sam2 for automatic video segmentation. We will be able to go back and firth select the exact frame we  want to work on. Cache inside a folder every individual. image with their generated mask. Each mask should have assigned an object with a name. it should save in the yolo format the segmentation the class and of the object type classification.

We will be using the model sam2 from ultralytics please check this link for reference https://docs.ultralytics.com/models/sam-2/

some of the requirements are the following

Move Back Forth in the frames
Conduct a Forward and Backwards Propagation from the selected frame.
Cache the frames in the file system.
Cache the Image-Encoder output from From SAM2 if using it. save it in a .pt file for later.
Save all the outputs in a YOLO dataset format.


create this on a folder called_data_engine
