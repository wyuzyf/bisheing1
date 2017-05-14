==========================================================================
DISCLAIMER: 
The demo software is designed in matlab and the result produced by the 
matlab code is not exactly same as the result presented by the paper since 
the result in the paper is produced originally with C++.  
The source code and data is made publicly available mainly for helping those 
who are interested in the implementation details of BPF.  Also for the sake 
of simplifying the implementation, the mixture particle filter is not used 
anymore and instead a set of indepedent particle filters is used. However,
this does not affect the core of BPF which is the construction of mixture 
proposals. Lastly, there is no guarantee for the presense of bugs in the code.  
==========================================================================

This directory contains the most recent version of the public distribution 
of BPF in Matlab.  There are a mex version of adaboost detector and a function 
of getting adaboost confidence.  
For copywright issues, please refer to a file called "COPYING.txt" in this 
directory. 

There are several subdirectories:
     BPF: the source code for BPF
external: code from external sources that are publicly available
			- KPMtools by Kevin Murphy 
              [ http://www.cs.ubc.ca/~murphyk/Software/index.html ]
			- lightspeed by Tom Minka 
              [ http://research.microsoft.com/~minka/software/lightspeed/ ]
    data: a sequence of image files  			
			

For the first time user:

1. Start Matlab 
2. >startup 
3. >test(TrackerBPF)

For details, please start reading BPF/@TrackerBPF/demo.m

NOTE: For the displaying purpose, color coded square boxes are drawn for tracked
targets.  However, the color information is only extracted from the black box
within a color coded square box.  The boxes that are close enough to be interacting
are colored in black. 


Acknowledgements:
Thank Wei-Lwun Lu for contributing his implementation of 
computing hsv color histograms using integral image techniques.  
Also thank Kevin Murphy and Tom Minka for their matlab tool box.
			
			
Kenji Okuma 
okumak[at]cs.ubc.ca