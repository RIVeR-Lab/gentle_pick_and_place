#include <point_cloud_proc/point_cloud_proc.h>

class ObjectClusteringServer{
    public:
        explicit ObjectClusteringServer(ros::NodeHandle n);
        // ~ObjectClusteringServer();

        bool clusterObjects(
            point_cloud_proc::TabletopClustering::Request& req,
            point_cloud_proc::TabletopClustering::Response& res);

    private:
        ros::NodeHandle nh;
        PointCloudProc* pcp_;

};


ObjectClusteringServer::ObjectClusteringServer(ros::NodeHandle n) : nh(n){

    std::string pkg_path = ros::package::getPath("pick_and_place");
    std::string pcp_config = pkg_path + "/config/pcp.yaml";
    pcp_ = new PointCloudProc(nh, true, pcp_config);

    ros::ServiceServer s_plan = nh.advertiseService("/pick_and_place/cluster_objects", &ObjectClusteringServer::clusterObjects, this);

    ros::waitForShutdown();

}

bool ObjectClusteringServer::clusterObjects(
            point_cloud_proc::TabletopClustering::Request& req,
            point_cloud_proc::TabletopClustering::Response& res){

    std::vector<point_cloud_proc::Object> objects;
    res.success = pcp_->clusterObjects(objects);
    if (res.success){
        res.objects = objects;
    }

    return res.success;
                
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_clustering_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ObjectClusteringServer server(nh);
    // ros::waitForShutdown();

}
