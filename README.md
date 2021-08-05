## [自动驾驶虚拟测试中的树木点云生成方法](https://kns.cnki.net/kcms/detail/detail.aspx?dbcode=cjfq&dbname=CJFDAUTO&filename=JSGG202111024)

### 摘要

自动驾驶汽车虚拟测试已成为自动驾驶或车路协同测试评价的一个重要手段，三维激光雷达数据模拟生成是自动驾驶汽车虚拟测试中的重要任务之一，目前多采用基于飞行时间原理的几何模型方法生成激光雷达三维点云数据，该方法生成点云实时性较差。布告牌是虚拟场景中常采用的树木建模方法，由于布告牌仅由两个矩形面片即八个三角形面片组成，直接采用布告牌方法生成的三维点云数据难以反映树木的真实空间信息。针对上述问题，提出了一种基于布告牌空间变换的快速树木三维点云生成方法。以布告牌的纹理图像为依据，根据纹理透明度获取树木二维平面点云分布，经二维树木点云的轮廓提取，结合树木结构的先验知识进行旋转、随机偏移和尺度变换，以更少的三角形面片数和更小的计算代价获得树木的三维点云数据。提出了一种空间直方图三维点云相似度评价方法，将三维点云空间量化为若干个子空间，获得三维点云的投影空间直方图，采用巴氏系数计算投影空间直方图相似度，以投影空间直方图加权相似度作为点云相似度评价值。实验结果表明，基于布告牌空间变换方法和几何模型方法生成的云杉等三种树木的三维点云数据的平均相似度在90%以上，且该方法生成树木点云的时间仅是几何模型法的1%，因此布告牌空间变换树木三维点云生成方法快速且准确，可以满足自动驾驶汽车虚拟测试的性能要求。

> **关键词**：自动驾驶汽车虚拟测试；三维点云；树木；布告牌；空间直方图

## Method for Generating Three-Dimensional Point Cloud of Trees in Autonomous Vehicle Virtual Testing

**Abstract**：Autonomous vehicle virtual testing has become an important method for autonomous driving or vehicle- road collaborative testing and evaluation. Virtual generation of LiDAR data is one of the most important tasks in autonomous vehicle virtual testing. At present, the geometric model method based on the time-of-flight principle is a common method to generate LiDAR 3D point cloud data, which may lead to poor performance for real- time generating point cloud. The billboard technology is often used to model trees in virtual scenes. Since the billboard is only composed of two rectangular meshes with eight triangles, the 3D point cloud data generated by the billboard directly can hardly reflect the real spatial information of trees. To solve the above problems, this paper proposes a fast 3D tree point cloud generation method based on the spatial transformation of the billboard. Based on the texture image of the billboard, the 2D point cloud of trees is obtained according to the texture transparency, and the boundary of the 2D tree point cloud is estimated. Combined with the prior knowledge of the tree structure, the rotation, random offset and scale transformation are carried out in turn. After the above operation, the 3D point cloud data of trees can be obtained with less triangle meshes and less computational cost. This paper also proposes a similarity evaluation method of the 3D point cloud based on spatial histogram. The 3D point cloud space is quantified into several subspaces, and the projection spatial histograms of the 3D point cloud are obtained, and the Bhattacharyya coefficient is adopted to calculate the similarity of two spatial histograms. The weighted similarity of projection spatial histogram is used as the evaluation of the point cloud similarity. The experimental results show that the average similarity between the 3D point cloud data of spruce and other three trees generated by the proposed method and by the geometric modeling methods is more than 90%. Compared with the geometric modeling methods, the method only takes about one percent of the time to generate 3D tree point cloud. The 3D point cloud of trees generation method based on the spatial transformation of the billboard is fast and accurate and can meet the performance requirements of autonomous vehicle virtual testing.

> **Key words**：autonomous vehicle virtual testing; 3D point cloud; tree; billboard; spatial histogram

##### 基于布告牌空间变换的快速树木三维点云生成方法框架图

![基于布告牌空间变换的快速树木三维点云生成方法框架图](/框架图.png)

![结果图](/results.png)

##### 空间直方图点云相似度评价方法

**优点**：空间直方图点云相似度评价方法考虑了点云的空间位置信息，更能反映点云分布的相似度。且避免了计算一个点集中所有点与另一点集所有点之间的距离，计算代价更小。

**主要原理**：将三维点云空间量化为若干个子空间，采用巴氏系数计算各投影子空间的空间直方图相似度，以投影空间直方图加权相似度作为点云相似度评价值。



**初始点云生成参考**：[GitHub - ptibom/Lidar-Simulator: Virtual Generation of Lidar Data for Autonomous Vehicles](https://github.com/ptibom/Lidar-Simulator)

**文章引用**:唐维军,徐琨,柳有权,夏悬.自动驾驶汽车虚拟测试中的树木点云生成方法[J].计算机工程与应用,2021,57(11):185-192.

**本文工作受到如下项目资助：**

> 封闭和半开放条件下智能车路系统测试评估与示范应用（国家重点研发计划，2018YFB1600800）
