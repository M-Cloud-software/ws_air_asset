# Single tarp detection with projected GPS coordinate

string  label             # always "tarp" for this pipeline
float32 confidence        # relative to min_pixels threshold (0–1 clamped)

# Bounding box in pixels
int32 bbox_x1
int32 bbox_y1
int32 bbox_x2
int32 bbox_y2

# Centroid in pixels
int32 pixel_cx
int32 pixel_cy

# Mean Sobel gradient magnitude inside the bounding box (quality metric)
float32 gradient_energy

# Projected GPS location of the detected object on the ground
float64 target_lat
float64 target_lon
