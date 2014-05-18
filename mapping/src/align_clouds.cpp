#include <iostream>
#include <string>
#include <numeric>

#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "point_defs.h"
#include "utils/hdf_utils.h"
#include "utils/cloud_utils.h"
#include "parameters.h"


namespace po = boost::program_options;

struct Options
{
    std::string pcd_tgt;
    std::string pcd_src;
    std::string h5_file;
    int icp_iters;
    float max_dist;
    bool debug;
    po::options_description desc;
};

int options(int ac, char ** av, Options& opts)
{
  // Declare the supported options.
  po::options_description desc = opts.desc;
  desc.add_options()("help", "Produce help message.");
  desc.add_options()
    ("pcd_tgt", po::value<std::string>(&opts.pcd_tgt)->required(), "pcd of cloud to align to")
    ("pcd_src", po::value<std::string>(&opts.pcd_src)->required(), "pcd of cloud to align")
    ("h5_file", po::value<std::string>(&opts.h5_file)->required(), "h5 file to alignment information to")
    ("icp_iters", po::value<int>(&opts.icp_iters)->default_value(5), "number of ICP iterations to run")
    ("debug", po::bool_switch(&opts.debug)->default_value(false), "debug flag")
    ("max_dist", po::value<float>(&opts.max_dist)->default_value(1.0), "maximum ICP correspondence distance")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  po::notify(vm);

  return 0;
}


// Define a new point representation for < x, y, z, curvature>
class XYZINormalPointRepresentation : public pcl::PointRepresentation <pcl::PointXYZINormal>
{
    using pcl::PointRepresentation<pcl::PointXYZINormal>::nr_dimensions_;
  public:
    XYZINormalPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 5;
    }
  
    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const pcl::PointXYZINormal&p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.intensity;
        out[4] = p.curvature;
    }
};


/*
float pair_align (const PointCloudWithNormals::Ptr cloud_src, const PointCloudWithNormals::Ptr tgt_cloud, PointCloudWithNormals::Ptr aligned_cloud, Eigen::Matrix4f &transform, int icp_iters, float max_dist)
{
    // TODO Determine whether need to set a custom point representation

    // Align

    XYZINormalPointRepresentation point_representation;
    std::vector<float> av = params().icp_coord_weights;
    std::cout << "weight: " << av[0] << " " << av[1] << " " << av[2] << " " << av[3] << " " << av[4] << std::endl;
    float alpha[5] = {av[0], av[1], av[2], av[3], av[4]};
    point_representation.setRescaleValues (alpha);

    pcl::IterativeClosestPointNonLinear<pcl::PointXYZINormal, pcl::PointXYZINormal> reg;
    reg.setTransformationEpsilon (1e-6);  // Change in transformation for (convergence) // PARAM
    reg.setMaxCorrespondenceDistance (max_dist);
      reg.setPointRepresentation (boost::make_shared<const XYZINormalPointRepresentation> (point_representation));

    PointCloudWithNormals::Ptr src_cloud(new PointCloudWithNormals());
    pcl::copyPointCloud (*cloud_src, *src_cloud);

    reg.setInputSource(src_cloud);
    reg.setInputTarget(tgt_cloud);

    reg.setMaximumIterations (2);  // Maximum iterations to run internal optimization // PARAM

    Eigen::Matrix4f T_i = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev;

    float prev_fitness_score = std::numeric_limits<float>::max();
    for (int i = 0; i < icp_iters; ++i)
    {
        reg.setInputSource(src_cloud);
        reg.align(*aligned_cloud);

        src_cloud = aligned_cloud;

        // Accumulate transformation
        T_i = reg.getFinalTransformation () * T_i;
        //std::cout << "T_" << i << std::endl;
        //std::cout << T_i << std::endl;

        // If the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing
        // the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        PCL_INFO("\titer: %d, fitness score: %f\n", i, reg.getFitnessScore());

        //if (reg.getFitnessScore() > prev_fitness_score)
        //{
            //prev_fitness_score = reg.getFitnessScore();
            //break;
        //}
        prev_fitness_score = reg.getFitnessScore();
    }

    transform = T_i;
    return reg.getFitnessScore();
}
*/


// NOTE cloud_in and cloud_out can't reference the same point cloud object
void filter_by_intensity(const PointCloudWithNormals::Ptr cloud_in, PointCloudWithNormals::Ptr cloud_out)
{
    cloud_out->clear();
    BOOST_FOREACH(PointTNormal p, cloud_in->points)
    {
        if (p.intensity > params().icp_min_intensity)
            cloud_out->push_back(p);
    }
}


float align_clouds(const PointCloudWithNormals::Ptr cloud_src, const PointCloudWithNormals::Ptr cloud_tgt, PointCloudWithNormals::Ptr aligned_cloud, Eigen::Matrix4f &final_transform, int iters, float max_dist, float tol, bool debug)
{
    final_transform = Eigen::Matrix4f::Identity();

    // Setup

    PointCloudWithNormals::Ptr src_cloud(new PointCloudWithNormals());
    PointCloudWithNormals::Ptr tgt_cloud(new PointCloudWithNormals());
    filter_by_intensity(cloud_src, src_cloud);
    filter_by_intensity(cloud_tgt, tgt_cloud);
    aligned_cloud = src_cloud;

    std::vector<float> normalized_errors;

    XYZINormalPointRepresentation point_representation;
    std::vector<float> av = params().icp_coord_weights;
    std::cout << "weight: " << av[0] << " " << av[1] << " " << av[2] << " " << av[3] << " " << av[4] << std::endl;
    float alpha[5] = {av[0], av[1], av[2], av[3], av[4]};
    point_representation.setRescaleValues (alpha);

    // First just estimate translation

    for (int k = 0; k < iters; k++)
    {
        // Estimate correspondences

        pcl::registration::CorrespondenceEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> correspondence_est;
        correspondence_est.setPointRepresentation(boost::make_shared<const XYZINormalPointRepresentation> (point_representation));
        correspondence_est.setInputSource(src_cloud);
        correspondence_est.setInputTarget(tgt_cloud);

        pcl::Correspondences all_correspondences;
        correspondence_est.determineCorrespondences(all_correspondences, max_dist);

        if (debug)
        {
            align_clouds_viz(src_cloud, tgt_cloud, aligned_cloud, all_correspondences);
        }

        if (all_correspondences.size() == 0)
        {
            std::cout << "No correspondences found!" << std::endl;
            throw;
        }

        std::vector<float> x_translations;
        std::vector<float> y_translations;
        std::vector<float> z_translations;
        // TODO May want to use these at some point
        std::vector<Eigen::Vector3f> translations;

        float normalized_error = 0;

        BOOST_FOREACH(pcl::Correspondence c, all_correspondences)
        {
            long idx_query = c.index_query;
            long idx_match = c.index_match;
            // TODO Not sure why this check is needed
            if (idx_query < 0 || idx_match < 0 || idx_query >= src_cloud->size() || idx_match >= tgt_cloud->size())
                continue;

            pcl::PointXYZINormal p_query = src_cloud->at(idx_query);
            pcl::PointXYZINormal p_match = tgt_cloud->at(idx_match);

            Eigen::Vector3f translation;
            translation << p_match.x - p_query.x, p_match.y - p_query.y, p_match.z - p_query.z;
            normalized_error += translation.norm();
            translations.push_back(translation);

            x_translations.push_back(p_match.x - p_query.x);
            y_translations.push_back(p_match.y - p_query.y);
            z_translations.push_back(p_match.z - p_query.z);
        }
        assert (translations.size() > 0);

        normalized_error = normalized_error / all_correspondences.size();
        normalized_errors.push_back(normalized_error);

        // Shift src_cloud;

        // Use median
        std::sort(x_translations.begin(), x_translations.end());
        std::sort(y_translations.begin(), y_translations.end());
        std::sort(z_translations.begin(), z_translations.end());
        float x_shift = x_translations[x_translations.size() / 2];
        float y_shift = y_translations[y_translations.size() / 2];
        float z_shift = z_translations[z_translations.size() / 2];

        // Use mean
        //float x_shift = std::accumulate(x_translations.begin(), x_translations.end(), 0.0f) / x_translations.size();
        //float y_shift = std::accumulate(y_translations.begin(), y_translations.end(), 0.0f) / y_translations.size();
        //float z_shift = std::accumulate(z_translations.begin(), z_translations.end(), 0.0f) / z_translations.size();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform(0, 3) = x_shift;
        transform(1, 3) = y_shift;
        transform(2, 3) = z_shift;

        pcl::transformPointCloud(*src_cloud, *aligned_cloud, transform);
        src_cloud = aligned_cloud;

        final_transform = transform * final_transform;

        PCL_INFO("iter %d, shift %f %f %f, error %f\n", k, x_shift, y_shift,
                z_shift, normalized_error);

        // Break if barely changing
        if (fabs(x_shift) < tol && fabs(y_shift) < tol && fabs(z_shift) < tol)
            break;

        //while (max_dist > 0.5) // PARAM
            //max_dist = max_dist / 2.0;
    }

    // Now that we've estimated the translation, estimate
    // a full transformation matrix using SVD

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> est;

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> correspondence_est;
    correspondence_est.setPointRepresentation(boost::make_shared<const XYZINormalPointRepresentation> (point_representation));
    correspondence_est.setInputSource(src_cloud);
    correspondence_est.setInputTarget(tgt_cloud);
    pcl::Correspondences all_correspondences;
    correspondence_est.determineCorrespondences(all_correspondences, 0.1);  // FIXME PARAM

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_src(new pcl::PointCloud<PointXYZ>());;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_tgt(new pcl::PointCloud<PointXYZ>());;
    pcl::copyPointCloud(*src_cloud, *xyz_cloud_src);
    pcl::copyPointCloud(*tgt_cloud, *xyz_cloud_tgt);

    // Shift point clouds to origin 
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*xyz_cloud_src, centroid);
    pcl::demeanPointCloud<pcl::PointXYZ>(*xyz_cloud_src, centroid, *xyz_cloud_src);
    pcl::demeanPointCloud<pcl::PointXYZ>(*xyz_cloud_tgt, centroid, *xyz_cloud_tgt);

    Eigen::Matrix4f transform;
    est.estimateRigidTransformation(*xyz_cloud_src, *xyz_cloud_tgt, all_correspondences, transform);

    // Combine the two transforms

    final_transform = transform * final_transform;

    // FIXME normalized_errors only error after translation

    return normalized_errors.back();
}


int main(int argc, char** argv)
{
    params().initialize();

    Options opts;
    if (options(argc, argv, opts))
        return 1;

    // Read in the PCD files

    PointCloudWithNormals::Ptr src_cloud(new PointCloudWithNormals());
    PointCloudWithNormals::Ptr tgt_cloud(new PointCloudWithNormals());
    load_cloud(opts.pcd_src, src_cloud);
    load_cloud(opts.pcd_tgt, tgt_cloud);

    PointCloudWithNormals::Ptr aligned_cloud(new PointCloudWithNormals());
    Eigen::Matrix4f transform;

    float score = align_clouds(src_cloud, tgt_cloud, aligned_cloud, transform, opts.icp_iters, opts.max_dist, params().icp_tol, opts.debug);

    if (opts.debug)
    {
        std::cout << "transform:" << std::endl;
        std::cout << transform << std::endl;

        //pcl::Correspondences correspondences;
        //align_clouds_viz<pcl::PointXYZINormal>(src_cloud, tgt_cloud, aligned_cloud, correspondences);

        // Don't write outputs
        return 0;
    }

    // TODO Move these into a class that can write to JSON

    transform.transposeInPlace();  // Since H5 uses row-major
    H5::H5File file(opts.h5_file, H5F_ACC_TRUNC);
    write_hdf_dataset(file, "/transform", transform, H5::PredType::NATIVE_FLOAT);
    write_hdf_attribute(file, "/transform", "fitness_score", &score);
    write_hdf_attribute(file, "/transform", "pcd_src", opts.pcd_src);
    write_hdf_attribute(file, "/transform", "pcd_tgt", opts.pcd_tgt);
    file.close();

    return 0;
}
