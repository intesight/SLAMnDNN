本模块生成俯视、透视、多视角功能的生成及对应lut的调用功能。对应的cmakelist只生成相应功能的.so库
build/目录下的build.sh脚本文件，编译生成liblut.so文件
若使用此库文件，需在执行目录的同级目录建立
lut/目录，用于存放.bin格式的lut表，及生成的中间参数

并将liblut.so文件、image_view_convert.h、common_parkinggo.h文件拷贝到需要调用的地方，并编写相应的cmakelist

具体的实现的功能及函数说，参考《俯视、透视生成接口文档.docx》


环视融合图尺寸：
621×720
    MG:
    对应真实物理尺寸：
    8000+1804+8000 × 8000+4612+8000
    <!-- 前后左右视野 -->
    <front_word_view>8000</front_word_view>

    <rear_world_view>8000</rear_world_view>

    <left_world_view>8000</left_world_view>

    <right_world_view>8000</right_world_view>

    GL8:
    <!-- 前后左右视野 -->
    <front_word_view>7678</front_word_view>

    <rear_world_view>7678</rear_world_view>

    <left_world_view>7964</left_world_view>

    <right_world_view>7964</right_world_view>


<!--MG 汽车宽度 -->
<car_world_width>1804</car_world_width>
<!-- 汽车长度 -->
<car_world_height>4612</car_world_height>
<!-- 后轴中心到车前距离 -->
<car_axle_coord>3594</car_axle_coord>

<!--GL8 汽车宽度 -->
<car_world_width>1878</car_world_width>
<!-- 汽车长度 -->
<car_world_height>5256</car_world_height>
<!-- 后轴中心到车前距离 -->
<car_axle_coord>4198</car_axle_coord>


<!--融合尺寸512×512，.xml做如下修改：-->
	<!--俯视展开左侧像素的高度-->
	<left_pixel_height>512</left_pixel_height>
	
	<!--尺寸改为512×512_40后：-->
	
		<!-- MG.xml前后左右视野 -->
		<front_word_view>8000</front_word_view>

		<rear_world_view>8000</rear_world_view>

		<left_world_view>9404</left_world_view>

		<right_world_view>9404</right_world_view>


		<!-- GL8.xml前后左右视野 -->
		<front_word_view>7678</front_word_view>

		<rear_world_view>7678</rear_world_view>

		<left_world_view>9367</left_world_view>

		<right_world_view>9367</right_world_view>



	<!--尺寸改为512×512_28后：-->
	
		<!-- MG.xml前后左右视野 -->
		<front_word_view>5023</front_word_view>

		<rear_world_view>5023</rear_world_view>

		<left_world_view>6427</left_world_view>

		<right_world_view>6427</right_world_view>

		<!-- Gl8.xml前后左右视野 -->
		<front_word_view>4700</front_word_view>

		<rear_world_view>4700</rear_world_view>

		<left_world_view>6389</left_world_view>

		<right_world_view>6389</right_world_view>


#------modified by liuli on 2018/11/23:-------------#
#//1.增加函数接口birdview2birdview（），俯视图坐标转平行四边形坐标
/*
 * author: liuli
 * date:
 * name:birdview2birdview
 * function: 函数功能： 给定原俯视图某一点的像素坐标(u,v)，输出对应拉伸后的俯视图像素坐标(u',v')。
 * parametes:
 *          IN： float stitchpoint[2] 原俯视图下像素坐标，坐标,[0] u坐标; [1] v坐标
 *          INOUT： float modifiedpoint[2] 拉伸后平行四边形像素坐标 [0] u‘坐标; [1] v’坐标
 * */
void birdview2birdview(IN float stitchpoint[2], INOUT float modifiedpoint[2]);
 

#//2.在.xml文件中增加：

<!-- 是否进行双线性插值 -->
<DEBUG_bilinear>0</DEBUG_bilinear>

分别调节平行四边形拉伸系数，选择是否进行双线性插值。


#//3.关于.so调用
通过修改.xml文件中参数，生成不同拉伸程度的lut表，用生成的lut表拼接图像。
#---------------------------------------------------#


#------modified by liuli on 2018/12/5:-------------#
#1.通过调整外参拉宽车位。


#------modified by liuli on 2018/12/20:-------------#
#1.yuv格式读取图片拼接，512×512,有双线性插值：6.5ms;无双线性插值：4.5ms。
#2.bmp格式读取图片拼接，512×512,有双线性插值：23ms;无双线性插值：6.5ms。

#------modified by liuli on 2019/01/10:-------------#
对给定角度的车位，拉伸成倾斜车位，并对相应车位点坐标进行转换
 1. 拼接的两种实现方式：
     (1). 调用  stitch_thetaLot(IN Mat raw_image, INOUT Mat stitch_fusion, IN float theta)接口,直接实现拼图(要求支持.xml相对路径和四路原始图排列顺序左上到右下为前后左右; 也可根据需要修改该函数内部代码，重新生成.so库调用);
     (2). 直接调用    get_stretchParms("./XML/parameter_GL8_08_23.xml", theta, stretch_coef, yCoef);
                     stitch_fusion = lutTable_generate_stitchFusion(front_image, back_image, left_image, right_image, true, true,
                                                                        true, true);
                                                                        两个函数，在get_stretchParms()函数输入角度实现拼图。
 2. 标注文件的坐标转换：
     (1). get_stretchParms("./XML/parameter_GL8_08_23.xml", theta, stretch_coef, yCoef);
          birdview2birdview(stitchpoint, modifiedpoint, stretch_coef, yCoef);


