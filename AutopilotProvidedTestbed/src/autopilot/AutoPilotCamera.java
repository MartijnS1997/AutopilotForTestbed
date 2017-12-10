package autopilot;

import java.util.*;

/**
 * Created by Martijn on 14/10/2017.
 * @author Martijn Sauwens
 *
 * note:
 * _____________________________________> (0, n: columns or x values)
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |/
 * (0, m : the rows or y values)
 */
public class AutoPilotCamera {

    public AutoPilotCamera(byte[] image, float horizontalAngleOfView, float verticalAngleOfView, int nbRows, int nbColumns) throws IllegalArgumentException{

        if(!isValidAngleOfView(horizontalAngleOfView) || ! isValidAngleOfView(verticalAngleOfView))
            throw new IllegalArgumentException(VIEWINGANGLE_EXCEPTION);

        this.nbRows = nbRows;
        this.nbColumns = nbColumns;
        this.horizAngleOfView = horizontalAngleOfView;
        this.verticalAngleOfView = verticalAngleOfView;
        this.setCameraImage(this.convertToCameraImage(image, nbRows, nbColumns));
    }


    /**
     * Loads the new image and searches for different colored cubes
     * @param imageByteArray the byte array containing the information for the cubes
     */
    public void loadNewImage(byte[] imageByteArray){
        if(!canHaveAsImageArray(imageByteArray))
            throw new IllegalArgumentException(INCOMPATIBLE_SIZE);
        CameraImage newImage = this.convertToCameraImage(imageByteArray, this.getNbRows(), this.getNbColumns());
        //System.out.println("looking for cubes");
        this.setCameraImage(newImage);
        List<Vector> cubesInPicture = this.getAllCubeCenters();
        //System.out.println(cubesInPicture);
        this.setCubesInPicture(cubesInPicture);


    }

    public Vector getCenterOfNCubes(int nbOfCubes) throws NoCubeException {
        List<Vector> cubesInPicture = this.getCubesInPicture();
//        System.out.println("nb of cubes in the picture: " + cubesInPicture.size());
//        System.out.println("cubes in picture: " + cubesInPicture);
        //first check if there are any cubes, if not return the center

        if(cubesInPicture.size() == 0){
            throw new NoCubeException();
        }

        float xOffset = this.getNbColumns()/2.0f;
        float yOffset = this.getNbRows()/2.0f;

        int cubeCounter = 0;
        Vector centerVector = new Vector();
        // the x value is the x pos of the cube, the y value is the y pos if the cube
        // and the z value is the size of the cube
        for(Vector cube: cubesInPicture) {
            if(cubeCounter == nbOfCubes){
                return centerVector;
            }
            float currentPixels = cube.getzValue();
            float prevTotalPixels = centerVector.getzValue();
            float newTotalPixels = currentPixels + prevTotalPixels;

            float xCenter = (centerVector.getxValue() * prevTotalPixels + cube.getyValue()*currentPixels)/(newTotalPixels);
            float yCenter = (centerVector.getyValue() * prevTotalPixels + cube.getxValue()*currentPixels)/(newTotalPixels);

            centerVector = new Vector(xCenter, yCenter, newTotalPixels);
        }

        // account for the offsets
        return new Vector(centerVector.getxValue() - xOffset, - centerVector.getyValue() + yOffset, centerVector.getzValue());

        /*if(cubesInPicture.size()==0)
            return new Vector();

        //then try to take the mean of the specified amount of cubes (weighted mean)
        float totalPixels = 0;
        Coordinates sumCoordinates = new Coordinates(0,0);
        //initialize a counter, break the loop if the counter is equal to the amount of cubes
        int counter = 0;
        for(Vector currentCube: cubesInPicture){
            if(counter == nbOfCubes)
                break;
            Coordinates currentWeightedCoord = new Coordinates(currentCube.getyValue(), currentCube.getxValue())
                    .scalarMult(currentCube.getzValue());
            sumCoordinates = sumCoordinates.sum(currentWeightedCoord);
            totalPixels += currentCube.getzValue();
            counter++;
        }

        //after the sum is completed return the weighted average
        Coordinates weightedCoord = sumCoordinates.scalarMult(1.0f/totalPixels);
        return new Vector(weightedCoord.getXCoordinate(), weightedCoord.getYCoordinate(), totalPixels);*/
    }



    //Todo adjust pixel classes such that the HSV values are correct!!!!!!!!!!!!!!!
    /**
     * locates all the cubes of a different color and returns them in order of size for each color
     * @return a list containing vectors where the x and y coordinates indicate the position on the screen
     *         and the z value contains the size of the cubes, the list is ordered from large
     */
    public List<Vector> locateCubes(){
        //coordinate is the H & S value of the pixel, the Coordinates in the list are the
        //coordinates of the pixels with the specific H & S values of the key
        HashMap<Coordinates ,List<Coordinates>> cubeMap = new HashMap<>();
        //System.out.println("identifying different cubes");
        findColoredPixels(cubeMap);
        //System.out.println("Sets of pixels" + cubeMap);

        //first two elements of the vector contain the x and the y position, the third the size of the cube
        HashMap<Coordinates, Vector> cubeCenterMap = new HashMap<>();

        //System.out.println("Calculating the mean of the cubes");
        calculateMeanCubes(cubeMap, cubeCenterMap);


        List <Vector> cubeOrderedList = new ArrayList<>();
        //System.out.println("sorting the cubes to size");
        cubeOrderedList.addAll(cubeCenterMap.values());
        cubeOrderedList.sort(new Comparator<Vector>() {
            /**
             * the list needs to be ordered from large to small so if V1 is larger than V2 it needs to
             * return a negative integer (normal sort is from small to large so we need to invert it)
             * @param v1 the first vector
             * @param v2 the second vector
             * @return -1 if V1 > V2, 0 if V1 == V2, 1 if V1 < V2
             */
            @Override
            public int compare(Vector v1, Vector v2) {
                //the z value will be always an integer value in this case
                return (int)(Math.signum(-v1.getzValue() + v2.getzValue()));
            }
        });
        //System.out.println("Positions of the cubes: " + cubeOrderedList);
        return cubeOrderedList;

    }

    /**
     * Calculates the mean coordinate of every different colored cube that is present in the cube map
     * @param cubeMap the map containing the pixels of one specific color of cube
     * @param cubeCenterMap the coordinates are the HSV values, the Vector contains for the
     *                      x and y values the mean position of one color cube, the z position is the
     *                      size of the given cube (the size = nb of pixels)
     */
    private void calculateMeanCubes(Map<Coordinates, List<Coordinates>> cubeMap, Map<Coordinates, Vector> cubeCenterMap){

        //the offsets needed for the transformation of the coordinates
        float xOffset = this.getNbColumns()/2.0f;
        float yOffset = this.getNbRows()/2.0f;
        

        //get the different colors
        for(Coordinates color: cubeMap.keySet()){
            //use the extracted key to get the list containing the pixels
            Coordinates colorSum = new Coordinates(0,0);
            List<Coordinates> currentColorList = cubeMap.get(color);
            for(Coordinates colorPos: currentColorList){
                //sum all the pixels
            	colorPos = new Coordinates(colorPos.getXCoordinate()-xOffset, -colorPos.getYCoordinate()+yOffset);
                colorSum = colorSum.sum(colorPos);
            }
            //the amount of colorPixels
            int nbColorPixels = currentColorList.size();
            //rescale the result to get the mean
            colorSum = colorSum.scalarMult(1.0f/nbColorPixels);

            //put the results in the map
            //the coordinates also need to be transformed for the autopilot, (0,0) is the middle of the screen
            cubeCenterMap.put(color, new Vector(colorSum.getXCoordinate(),
                    colorSum.getYCoordinate(), nbColorPixels));
        }
    }

    /**
     * Finds all colored pixels and stores them in the provided cubemap
     * @param cubeMap the keys are the colors of the cube and the list contains the coordinates
     *                of the pixels with the same color
     */
    private void findColoredPixels(Map<Coordinates, List<Coordinates>> cubeMap){

        int nbRows = this.getNbRows();
        int nbColumns = this.getNbColumns();

        CameraImage cameraImage = this.getCameraImage();

        Coordinates zeroCoord = new Coordinates(0.0f, 0.0f);

        for(int i = 0; i != nbRows; i++){
            for(int j = 0;  j!= nbColumns; j++){
                Pixel currentPixel = cameraImage.getElementAtIndex(i, j);
                float[] HSVPixel = currentPixel.convertToHSV();
                float H = HSVPixel[0];
                if(Float.isNaN(H))
                    H = 0.0f;
                float S = HSVPixel[1];
                //System.out.println("H value:" +  H + " , S value:" + S);

                Coordinates key = new Coordinates(H, S);

                //check if it is not just a background pixel
                if(key.equals(zeroCoord))
                    continue; // if it is a zero coord just goto the next iteration

                if(cubeMap.get(key) == null){
                    //the j values are the x coordinates and the i values are the y coordinates
                    Coordinates listValue = new Coordinates(j, i);
                    //create the list to store same colored pixels
                    List<Coordinates>  value = new ArrayList<>();
                    //add the current value to the list
                    value.add(listValue);
                    //add the list as value to the map
                    cubeMap.put(key, value);
                }else{
                    //get the list of the corresponding color
                    List<Coordinates> values = cubeMap.get(key);
                    //add the new pixel with the corresponding color
                    values.add(new Coordinates(j, i));
                }
            }
        }

        //at the end of the for loop all te pixels are iterated and the cube map is filled with all the visible cubes
    }

    /**
     * A private class of immutable Coordinates
     * @author Martijn Sauwens
     */
    private class Coordinates{

        Coordinates(float x, float y){
            xCoordinate = x;
            yCoordinate = y;
        }

        /**
         * calculates the sum of two coordinates
         * @param other the other coordinates to sum
         * @return a new coordinates object containing the sum of both coordinates
         */
        public Coordinates sum(Coordinates other){
            float x = this.xCoordinate + other.xCoordinate;
            float y = this.yCoordinate + other.yCoordinate;

            return new Coordinates(x, y);
        }

        /**
         * Method that rescales the coordinates with a given scalar
         * @param scalar the scalar to multiply the coordinates with
         * @return  a new coordinates object containing the rescaled coordinates
         *          Coordinates( scalar * x, scalar * y)
         */
        private Coordinates scalarMult(float scalar){
            float x = this.getXCoordinate();
            float y = this.getYCoordinate();

            return new Coordinates(x * scalar, y*scalar);
        }

        private float getXCoordinate() {
            return xCoordinate;
        }

        private float getYCoordinate(){
            return yCoordinate;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Coordinates)) return false;

            Coordinates that = (Coordinates) o;
            float errorMargin = 0.1f;
            return Pixel.isEqualFloat(this.getXCoordinate(), that.getXCoordinate(), errorMargin)&&
                    Pixel.isEqualFloat(this.getYCoordinate(), that.getYCoordinate(), errorMargin);

        }

        @Override
        public int hashCode() {
            int result = (getXCoordinate() != +0.0f ? Float.floatToIntBits(getXCoordinate()) : 0);
            result = 31 * result + (getYCoordinate() != +0.0f ? Float.floatToIntBits(getYCoordinate()) : 0);
            return result;
        }

        @Override
        public String toString() {
            return "(" +  xCoordinate +
                    " ; "+ yCoordinate +
                    ")";
        }

        private float xCoordinate;
        private float yCoordinate;
    }

    /**
     * Converts an array of bytes containing the RGB color scheme to an array consisting of pixels
     * @param image the byte array containing the data for the pixels
     * @return an array containing the pixels of the image
     * byte array ={ R1, G1, B1, R2, G2, B2, R3, G3, B3} then
     * pixel array = { P1(RGB), P2(RGB), P3(RGB)}
     */
    public Pixel[] convertToPixelArray(byte[] image){
        int imageArraySize = image.length;
        int pixelArraySize = imageArraySize/NB_OF_BYTES_IN_PIXEL;
        Pixel[] pixelArray = new Pixel[pixelArraySize];

        //loop through the indices of the pixel array
        for(int pixelIndex = 0; pixelIndex != pixelArraySize; pixelIndex++){
            byte[] onePixel = new byte[NB_OF_BYTES_IN_PIXEL];

            // the byte array is three times longer than the pixel array, place every three bytes in one pixel
            for(int byteExtraIndex = 0; byteExtraIndex != NB_OF_BYTES_IN_PIXEL; byteExtraIndex++){

                onePixel[byteExtraIndex] = image[pixelIndex*NB_OF_BYTES_IN_PIXEL + byteExtraIndex];
            }
            pixelArray[pixelIndex] = new Pixel(onePixel);
        }

        return pixelArray;
    }


    /**
     * Converts an image byte array to a 2D array containing the pixels of the image
     * @param byteImage the bytes containing the pixels of the image
     * @param nbRows the number of rows of the pixel array
     * @param nbColumns the number cf columns of the pixel array
     * @return a 2D array containing the pixels of the image
     */
    public CameraImage convertToCameraImage(byte[] byteImage, int nbRows, int nbColumns){
        Pixel[] pixelArray = this.convertToPixelArray(byteImage);
        return new CameraImage(pixelArray, nbRows, nbColumns);
    }

    /**
     * Checks if the angle of view is valid
     * @param angle the angle to check
     * @return true if and only if the view is within range (0°, 180°]
     */
    public boolean isValidAngleOfView(float angle){
        //System.out.println("viewing Angle: " + angle);
        return angle > 0.0f && angle <= Math.PI;
    }

    public CameraImage getCameraImage(){
        return this.cameraImage;
    }

    public void setCameraImage(CameraImage cameraImage){
        this.cameraImage = cameraImage;
    }

    /**
     * Checks if the given image array can be set as as image array
     * @param imageArray the cameraImage to be checked
     * @return true if and only if the cameraImage is the right size
     */
    public boolean canHaveAsImageArray(byte[] imageArray){
       return this.getNbRows()*this.getNbColumns() == imageArray.length/NB_OF_BYTES_IN_PIXEL;
    }

    /**
     * Getter for the number of rows in the camera image
     * @return the number of rows in a picture
     */
    public int getNbRows() {
        return nbRows;
    }

    /**
     * Getter for the number of columns in the camera image
     * @return the number of columns in a picture
     */
    public int getNbColumns() {
        return nbColumns;
    }

    /**
     * @author anthonyrathe
     * @return
     */
    public Vector getDestination(){
        return this.destination;
    }

    /**
     * @author anthonyrathe
     * @return
     */
    public void setDestination(Vector destination){
        this.destination = destination;
    }


    /**
     * get pixels that qualify the recognition criteria
     * @return the total qualified pixels
     */
    public int getTotalQualifiedPixels() {
        return totalQualifiedPixels;
    }

    /**
     * the total amount of qualified pixels
     * @param totalQualifiedPixels an integer containing the total amount of qualified pixels
     */
    public void setTotalQualifiedPixels(int totalQualifiedPixels) {
        this.totalQualifiedPixels = totalQualifiedPixels;
    }

    /**
     * Getter for the list that contains the cubes of different color that could be located in the picture
     * @return a list containing the on screen center coordinates of the cubes and the nb of pixels
     */
    public List<Vector> getCubesInPicture() {
        return cubesInPicture;
    }

    /**
     * Setter for the list tht contains the cubes of different color
     * @param cubesInPicture the list containing vectors with the first two being the onscreen coordinates of the cubes
     *                       and the final element being their size
     */
    public void setCubesInPicture(List<Vector> cubesInPicture) {
        this.cubesInPicture = cubesInPicture;
    }

    /*
                Variables
                 */
    private CameraImage cameraImage;

    /**
     * The number of pixel rows the pixel image contains (immutable)
     */
    private int nbRows;
    /**
     * The number of pixel columns the image contains (immutable)
     */
    private int nbColumns;
    /**
     * horizontal viewing angle of the drone (immutable)
     */
    private float horizAngleOfView;
    /**
     * vertical viewing angle of the drone (immutable)
     */
    private float verticalAngleOfView;

    private Vector destination;
    private int totalQualifiedPixels;
    private List<Vector> cubesInPicture = new ArrayList<>();


    /*
    Constant tags
     */
    public final static String RED_PIXEL = "red";
    public final static String BLUE_PIXEL = "blue";
    public final static String GREEN_PIXEL = "green";
    /*
    Constants
     */
    private final static int NB_OF_BYTES_IN_PIXEL = 3;
    private final static float RED_H_VALUE = 0.0f;
    private final static float RED_S_VALUE = 1.0f;
    private final static float Z_AXIS_V_VALUE = 0.7f;
    private final static float EPSILON  = 1E-2f;


    /*
    Error Messages
     */
    public final static String IO_EXCEPTION = "Something went wrong reading the image";
    public final static String VIEWINGANGLE_EXCEPTION = "the viewing angle is out of range (0, PI]";
    public final static String ILLEGAL_SIZE = "The byte array cannot be converted to a 2D pixel array";
    public final static String INCOMPATIBLE_SIZE = "the sample size is not a multiple of the dimensions of the array";

    public List<Vector> getAllCubeCenters(){
        CameraImage currentImage = this.getCameraImage();
        // first get the center coordinates
        Map<Coordinates, Vector> cubeCenterMap = getDifferentCubeMap(currentImage);

        // then get the ordered list, small to large
        List<Vector> orderedList = new ArrayList<>(cubeCenterMap.values());
        orderedList.sort(new Comparator<Vector>() {
            @Override
            public int compare(Vector o1, Vector o2) {
                return -(int) Math.signum(o1.getzValue() - o2.getzValue());
            }
        });

        return orderedList;

    }

    private Map<Coordinates, Vector> getDifferentCubeMap(CameraImage currentImage) {
        Map<Coordinates, Vector> colorCenter =  new HashMap<>();


        for(int i = 0; i !=this.getNbRows(); i++){
            for(int j = 0; j!= this.getNbColumns(); j++) {
                Pixel currentPixel = currentImage.getElementAtIndex(i, j);
                float[] HSVPixel = currentPixel.convertToHSV();
                float H = HSVPixel[0];
                float S = HSVPixel[1];
                float V = HSVPixel[2];


                // H can be NaN
                if (Float.isNaN(H)) {
                    H = 0.0f;
                }
                //if it is a white pixel ignore it
                if( H == 0.0f &&S == 0.0f&&V==1.0f)
                    continue;

                Coordinates HS = new Coordinates(H, S);

                Vector entry = colorCenter.get(HS);

                // if the HS value is not yet part of the map, add it
                if (entry == null) {
                    colorCenter.put(HS, new Vector(i, j, 1.0f));
                //if it is already present, calculate the new mean value for the position
                }else{
                    // first load the old values for the entry
                    float oldIMean = entry.getxValue();
                    float oldJMean = entry.getyValue();
                    float oldSize = entry.getzValue();

                    //calculate the new mean
                    float iNumerator = oldIMean*oldSize + i;
                    float jNumerator = oldJMean*oldSize + j;

                    float denominator = oldSize + 1;

                    Vector newMean = new Vector(iNumerator/denominator, jNumerator/denominator, denominator);

                    colorCenter.put(HS, newMean);
                }
            }
        }
        //System.out.println("HS value and position on screen: " + colorCenter);
        //System.out.println("Encountered colors: " + HSVSet);
        return colorCenter;
    }
}
