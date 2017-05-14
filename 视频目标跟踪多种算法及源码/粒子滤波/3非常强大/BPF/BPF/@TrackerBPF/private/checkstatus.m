function status = checkstatus(obj, particle)
% PURPOSE : Check the status of particles
% INPUT :  - obj      = @TrackerBPF object
%          - particle = a set of particles to check
% OUTPUT : - status   = status for all particles (DEAD or ALIVE)
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

num_pc = length(particle);
status = zeros(1,num_pc);
% compute overlap between a box and an entire image
A = [1 1 obj.img_width obj.img_height];

for i = 1:num_pc,
  width = obj.box_width*particle(3,i);
  height = obj.box_height*particle(3,i);
  frame = particle(1:2,i) - [width*.5; height*.5];
  B = round([frame(1) frame(2) width height]);
  [overlap normOverlap] = rectintC(A,B);
  % particle is not totally overlapped with image region, then it is dead 
  if normOverlap < 1
    status(i) = false;
  else
    status(i) = true;
  end
end