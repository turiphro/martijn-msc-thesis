# martijn-msc-thesis
3D reconstruction with specular objects -- MSc thesis @ UCL

CGVI master thesis - Structure from Visibility: Finding low-textured and reflective occluders in outdoor scenes using visibility and occlusion

_The problem of geometry reconstruction is challenging and the diversity in scenes asks for solid solutions. The literature on geometry reconstruction is comprehensive and many approaches have been tried, with varying degrees of success. Most approaches are based on some form of photo-consistency, that is, they rely on_directly_observing all objects that are being reconstructed. However, this casts constraints on the surface properties of the objects. Surfaces often need to be either uniformly coloured (low-textured) and well differentiable from the background, or they need to have clear and rich textures and Lambertian-like properties. In practice, these constraints are not always met. Therefore, many algorithms fail on certain kinds of objects._

_We propose a new approach based on_indirectly_observing objects using visibility information. Based on the intuition that detection of features from certain view points and absence of detection in others can provide clues on where occluder objects may exist, we developed two algorithms; one based on visibility information alone, and one using both visibility and occlusion (i.e., absence of visibility). An occupancy grid is altered iteratively based on this information. Feature estimation is obtained by using reliable Structure from Motion tools and no pixel pair-wise guesswork needs to be done._

The proposed method has been shown to provide reasonable results for scenes containing low-textured or reflective objects. For good results, enough view points need to be provided (e.g., by walking around an object) and background objects containing a decent amount of features need to be present. Visibility information helps carving much of the empty space for which reliable features on background objects are extracted, but leaves space with less visibility rays partly labelled as occupied. Using both visibility and occlusion cues (and default world view `unknown') improves results. Furthermore, our Visibility-Occlusion algorithm outperforms the recent dense reconstruction algorithm CMVS/PMVS on the tested reflective objects. Visibility and occlusion therefore appear useful cues for geometry reconstruction.

![example image](https://raw.githubusercontent.com/turiphro/martijn-msc-thesis/master/doc/img/car_and_wall1_sparse.png)

![image 2](https://raw.githubusercontent.com/turiphro/martijn-msc-thesis/master/doc/img/memorial_carve2.png)
