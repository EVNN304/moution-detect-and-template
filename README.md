# moution-detect-and-template
This repository demonstrates the joint work of motion detection by subtracting 3 frames and object correlation by templates
The program implements:
1. Motion detection
2. Transmission of motion coordinates to the block of the correlation program
3. Cutting a fragment from the main image and comparing with templates
4. Selecting the maximum probability over all patterns and comparing with the set threshold
5. Draw the box if the threshold is passed
