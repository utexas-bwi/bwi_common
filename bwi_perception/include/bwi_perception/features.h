#ifndef BWI_PERCEPTION_FEATURES_H
#define BWI_PERCEPTION_FEATURES_H
namespace bwi_perception {

    pcl::PointCloud<pcl::Normal>::Ptr
    computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double normals_search_radius) {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation <pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(normals_search_radius);

        // Compute the features
        ne.compute(*cloud_normals);
        // Check for undefined values
        for (int i = 0; i < cloud_normals->points.size(); i++) {
            if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i])) {
                PCL_WARN("normals[%d] is not finite\n", i);
            }
        }

        return cloud_normals;
    }

    std::vector<double> computeCVFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double normals_search_radius) {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud, normals_search_radius);
        ROS_INFO("Computing CVFH...");

        // Create the CVFH estimation class, and pass the input dataset+normals to it
        pcl::CVFHEstimation <pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfh;
        cvfh.setInputCloud(cloud);
        cvfh.setInputNormals(cloud_normals);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        cvfh.setSearchMethod(tree);

        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

        cvfh.compute(*vfhs);

        // Convert into normalized vector
        int histogram_sum = 0;
        std::vector<double> histogram_double_vector(308);
        for (int i = 0; i < histogram_double_vector.size(); i++) {
            histogram_double_vector[i] = vfhs->points[0].histogram[i];
            histogram_sum += vfhs->points[0].histogram[i];
        }
        for (int i = 0; i < histogram_double_vector.size(); i++) {
            histogram_double_vector[i] /= (double) histogram_sum;
        }

        return histogram_double_vector;
    }

    std::vector<double>
    computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double neighbor_radius, double donormal_search_radius) {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud, donormal_search_radius);
        ROS_INFO("Computing FPFH...");

        // Create the FPFH estimation class, and pass the input dataset+normals to it
        pcl::FPFHEstimation <pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(cloud_normals);
        // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);

        fpfh.setSearchMethod(tree);

        // Output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());


        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        fpfh.setRadiusSearch(neighbor_radius);

        // Compute the features
        fpfh.compute(*fpfhs);

        // Convert into normalized vector
        int histogram_sum = 0;
        std::vector<double> histogram_double_vector(308);
        for (int i = 0; i < histogram_double_vector.size(); i++) {
            histogram_double_vector[i] = fpfhs->points[0].histogram[i];
            histogram_sum += fpfhs->points[0].histogram[i];
        }
        for (double &i : histogram_double_vector) {
            i /= (double) histogram_sum;
        }

        return histogram_double_vector;

        // fpfhs->points.size() should have the same size as the input cloud->points.size ()*
        // return fpfhs;      Put this back in the function definition if returning fpfhs---> pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    }

    std::vector <std::vector<std::vector < uint>> >
    computeRGBColorHistogram(pcl::PointCloud<pcl::PointXYZRGB>
    &cloud,
    int dim
    ) {

    //a 3D array
    std::vector <std::vector<std::vector < uint>> >
    hist3;

    hist3.
    resize(dim);
    for (
    int i = 0;
    i<dim;
    i++) {
    hist3[i].
    resize(dim);
    for (
    int j = 0;
    j<dim;
    j++) {
    hist3[i][j].
    resize(dim);
    std::fill( hist3[i][j]
    .

    begin(), hist3[i][j]

    .

    end(),

    0 );
}
}

int cloud_size = cloud.points.size();
for (
int i = 0;
i<cloud_size;
i++) {
// Max value of 255. We want 256 because we want the rgb division to result in
// a value always slightly smaller than the dim due to index starting from 0
double round = (double) 256 / dim;
int r = (int) ((double) (cloud.points[i].r) / round);
int g = (int) ((double) (cloud.points[i].g) / round);
int b = (int) ((double) (cloud.points[i].b) / round);
hist3[r][g][b]++;
}

return
hist3;
}

}
#endif //BWI_PERCEPTION_FEATURES_H
