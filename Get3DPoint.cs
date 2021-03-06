public class Laser {
    private int laserId;
    private Ray ray;
    private RaycastHit hit;
    private bool isHit;
    private float rayDistance;
    private float verticalAngle;
    private GameObject parentObject;
    private RenderLine lineDrawer;
    private float offset;

    public Laser(GameObject parent, float verticalAngle, float distance, float offset, GameObject lineDrawer, int laserId)
    {
        this.laserId = laserId;
        parentObject = parent;
        this.offset = offset;
        this.verticalAngle = verticalAngle;
        rayDistance = distance;
        this.lineDrawer = lineDrawer.GetComponent<RenderLine>();
        lineDrawer.transform.position = parentObject.transform.position + (parentObject.transform.up * offset);
        ray = new Ray();
        UpdateRay();
    }

    // Should be called from Update(), for best performance.
    // This is only visual, for debugging.
    public void DrawRay()
    {
        if (isHit)
        {
            lineDrawer.DrawLine(hit.point);
        }
        else
        {
            lineDrawer.DrawLine(ray.GetPoint(rayDistance));
        }
    }

    public void DebugDrawRay()
    {
        float distance = rayDistance;
        if (isHit)
        {
            distance = hit.distance;
        }
        Debug.DrawRay(ray.origin, ray.direction * distance, Color.red);
    }

    // Should be called from FixedUpdate(), for best performance.
    public RaycastHit ShootRay()
    {
        // Perform raycast
        UpdateRay();

        isHit = Physics.Raycast(ray, out hit, rayDistance);
        DrawRay();

        if (isHit)
        {
            return hit;
        }
        return new RaycastHit();
    }

    public void ShootRayWithOut()
    {
        UpdateRay();
        DrawRay();
    }
    

    // Update existing ray. Don't create 'new' ray object, that is heavy.
    private void UpdateRay()
    {
        Quaternion q = Quaternion.AngleAxis(verticalAngle, Vector3.right);
        Vector3 direction = parentObject.transform.TransformDirection(q * Vector3.forward);
        ray.origin = parentObject.transform.position + (parentObject.transform.up * offset);
        ray.direction = direction;
    }

    public Ray GetRay()
    {
        return ray;
    }

    public float GetVerticalAngle()
    {
        return verticalAngle;
    }

    public int GetLaserId()
    {
        return laserId;
    }

    public RenderLine GetRenderLine()
    {
        return lineDrawer;
    }
}



public void Get2DPoint(){

	RaycastHit hit = laser.ShootRay();
	float distance = hit.distance;
	if (distance != 0) // Didn't hit anything, don't add to list.
	{
	    float verticalAngle = laser.GetVerticalAngle();
	    if (hit.collider.CompareTag("Tree"))
	    {
	        GetTexture2DPointCloud(hit, verticalAngle, laser);
	    }
	    else
	    {
	        hits.AddLast(new SphericalCoordinate(distance, verticalAngle, horizontalAngle,
	            hit.point, laser.GetLaserId(), hit.collider.tag));
	    }
	}
}




// ???????????????????????????2D????????????
public void GetTexture2DPointCloud(RaycastHit hit, float verticalAngle, Laser laser)
{
    Vector2 pixelUV = hit.textureCoord;
    //?????????????????????????????????
    pixelUV.x *= treeTexture.width;
    pixelUV.y *= treeTexture.height;
                        

    if (treeTexture.GetPixel((int) pixelUV.x, (int) pixelUV.y).a == 1)
    {
        hits.AddLast(new SphericalCoordinate(hit.distance, verticalAngle, horizontalAngle,
            hit.point,
            laser.GetLaserId(), "Tree"));
        treePointsData.Add(new[]
        {
            hit.point.x.ToString(), hit.point.y.ToString(), hit.point.z.ToString(), hit.distance.ToString(), laser.GetLaserId().ToString()

        });
    }
}



private List<string[]> Get3DPoint(Vector3 point, float theta, Vector3 CenterPostion){

	for (float angle = -90f; angle < 90f; angle += theta)
	{
	    Vector3 worldCoodinate = RotateRound(point.GetWorldCoordinate(), CenterPostion, Vector3.up, angle);
	    worldCoodinate = MatrixTransfer(worldCoodinate);
	    savePointData.Add(new []{worldCoodinate.x.ToString(), worldCoodinate.y.ToString(), worldCoodinate.z.ToString()});
	}

}



private Vector3 MatrixTransfer(Vector3 originalPoint)
{
    double[,] originValue = { { originalPoint.x, originalPoint.y, originalPoint.z, 1 } };  // ????????????


    double[,] scaleValue, rotateValueX, rotateValueY;

    //????????????xz??????,???????????????????????????
    if (originalPoint.y < 1.68)
    {
        double rotateThetaX = RandOfDistribution(distributions, 0.045);    // ???X???????????????
        double rotateThetaY = RandOfDistribution(distributions, 0.03);     // ???Y???????????????
        double scaleRatio = RandOfDistribution(distributions, 0.0005);      // X,Z??????????????????
        rotateValueX = RotateX(rotateThetaX);                       // ???X?????????????????????
        rotateValueY = RotateY(rotateThetaY);                       // ???Y?????????????????????
        scaleValue = Scale(ratioX: 1 - scaleRatio, ratioZ: 1 - scaleRatio);   // X???Z????????????
    }
    else
    {
        double rotateThetaX = RandOfDistribution(distributions, 0.045);    // ???X???????????????
        double rotateThetaY = RandOfDistribution(distributions, 0.03);     // ???Y???????????????
        double scaleRatio = RandOfDistribution(distributions, 0.0005);      // X,Z??????????????????
        rotateValueX = RotateX(rotateThetaX);                       // ???X?????????????????????
        rotateValueY = RotateY(rotateThetaY);                       // ???Y?????????????????????
        scaleValue = Scale(ratioX: 1, ratioZ: 1);   // X???Z?????????????????????

    }


    var matrixOrigin = DenseMatrix.OfArray(originValue);  // ?????????
    var matrixScale = DenseMatrix.OfArray(scaleValue);
    var matrixRotateX = DenseMatrix.OfArray(rotateValueX);  // Y???????????? ??????
    var matrixRotateY = DenseMatrix.OfArray(rotateValueY);  // Y???????????? ??????
    var matrixResult = matrixOrigin * matrixScale * matrixRotateX * matrixRotateY ;
    
    var vectorResult = new Vector3((float)matrixResult.At(0, 0), (float)matrixResult.At(0, 1), (float)matrixResult.At(0, 2));
    return vectorResult;
}



   /// <summary>
    /// ??????????????????????????????
    /// </summary>
    /// <param name="distribution"></param>
    /// <param name="range">????????????-range, range???</param>
    /// <returns></returns>
    public double RandOfDistribution(Distributions distribution, double range)
    {
        double offsetTheta = 0;
        switch (distribution)
        {
            case Distributions.Gauss:
                var randGauss = new Normal(2, 50);
                offsetTheta = randGauss.RandomSource.NextDouble() * range * 2 - range;
                break;
            case Distributions.Binomial:
                var randBinomial = new Binomial(0.1, 2);
                offsetTheta = randBinomial.RandomSource.NextDouble() * range * 2 - range;
                break;
            case Distributions.Poisson:
                var randPoisson = new Poisson(20);
                //Debug.Log("pission");
                offsetTheta = randPoisson.RandomSource.NextDouble() * range * 2 - range;
                break;
            case Distributions.StuentT:
                var randT = new StudentT();
                offsetTheta = randT.RandomSource.NextDouble() * range * 2 - range;
                break;
               
        }
        return offsetTheta;
    }
    
    /// <summary>
    /// ???X??????????????????
    /// </summary>
    /// <param name="theta"></param>
    /// <returns></returns>
    private double[,] RotateX(double theta = 0)
    {
        double rad = theta * Math.PI / 180;
        double[,] matrixX = { {1,          0    ,       0      , 0 },
            {0,  Math.Cos(rad), Math.Sin(rad), 0 },
            {0, -Math.Sin(rad), Math.Cos(rad), 0 },
            {0,          0    ,       0      , 1 } };
        return matrixX;
    }

    /// <summary>
    /// ???Y?????????????????????
    /// </summary>
    /// <param name="theta"></param>
    /// <returns></returns>
    private double[,] RotateY(double theta = 0)
    {
        double rad = theta * Math.PI / 180;
        double[,] matrixY = { {Math.Cos(rad), 0, -Math.Sin(rad), 0 },
            {       0     , 1,        0      , 0 },
            {Math.Sin(rad), 0,  Math.Cos(rad), 0 },
            {       0     , 0,        0      , 1 } };
        return matrixY;
    }
    
    /// <summary>
    /// ???Z?????????????????????
    /// </summary>
    /// <param name="theta"></param>
    /// <returns></returns>
    private double[,] RotateZ(double theta = 0)
    {
        double rad = theta * Math.PI / 180;
        double[,] matrixZ = { {Math.Cos(rad), -Math.Sin(rad),  0, 0 },
            {Math.Sin(rad),  Math.Cos(rad),  0, 0 },
            {      0      ,        0      ,  1, 0 },
            {      0      ,        0      ,  0, 1 } };
        return matrixZ;
    }
    
    /// <summary>
    /// ????????????
    /// </summary>
    /// <param name="ratioX"></param>
    /// <param name="ratioY"></param>
    /// <param name="ratioZ"></param>
    /// <returns></returns>
    private double[,] Scale(double ratioX = 1, double ratioY = 1, double ratioZ = 1)
    {
        double[,] matrixS= { {ratioX,   0   ,   0   , 0 },
            {  0   , ratioY,   0   , 0 },
            {  0   ,   0   , ratioZ, 0 },
            {  0   ,   0   ,   0   , 1 } };
        return matrixS;
    }