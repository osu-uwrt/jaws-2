tGetting started with the Arbotix-M

1. First off: Install the Arduino suite:

On ubuntu this is :

sudo apt-get update

Followed by:

sudo apt-get intall arduino arduino-core

**This can also be installed via zip from their website. This method is a lot e$

2. Download the Arbotix libraries and hardware drivers

In one easy click:
https://github.com/trossenrobotics/arbotix/archive/master.zip

3. You must run the Arduino IDE. Just start it up, run throught eh first time prompts, and shut it down.

4. Move the files inside the zip file to 

/YOUR_USERNAME/sketchbook/

This folder should now contain the folders:

ArbotiX Sketches, hardware, libraries

5. At this point you're done.

6. To upload a sketch onto the ArbotiX, you must first upload the "ros" sketch in the ArbotiX Sketches sketchbook.
	**When you upload a sketch, make sure the board is set to "Arbotix"

7. Also to run DynaManager, you must have the ros sketch uploaded.

8. If you get the error "avrdude not responding", restart your COMPUTER, not virtual machine. Try and try again until it works.

