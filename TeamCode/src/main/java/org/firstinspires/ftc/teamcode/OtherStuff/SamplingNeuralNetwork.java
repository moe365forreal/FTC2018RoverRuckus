package org.firstinspires.ftc.teamcode.OtherStuff;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.support.annotation.ColorInt;

import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class SamplingNeuralNetwork {
    public static String MODEL_FILE_PATH = "lrc_mineralrecognition.tflite";
    public static final int BYTE_SIZE_OF_FLOAT = 4;

    private MappedByteBuffer loadModelFile(Context activity) throws IOException {
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd(MODEL_FILE_PATH);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    public String runPrediction(Interpreter interpreter, String assetImagePath, Context context) {
        ByteBuffer inputBuffer = null;
        try {
            // get input stream
            InputStream ims = context.getAssets().open(assetImagePath);
            // load image as Drawable
            Bitmap image = Bitmap.createScaledBitmap(BitmapFactory.decodeStream(ims), 80, 53, false);
            image = Bitmap.createBitmap(image, 0, 15, 80, 29);
            inputBuffer = ByteBuffer.allocateDirect(BYTE_SIZE_OF_FLOAT * image.getWidth() * image.getHeight());
            inputBuffer.order(ByteOrder.nativeOrder());
            int i = 0;
            for (int y = 0; y < image.getHeight(); y++) {
                for (int x = 0; x < image.getWidth(); x++) {
                    @ColorInt int rgb = image.getPixel(x, y);
                    inputBuffer.putFloat((65536 * (Color.red(rgb))) + (256 * (Color.blue(rgb))) + Color.green(rgb));
                }
            }
        } catch (IOException ex) {
            return "ERROR READING IN IMAGE - IOEXCEPTION";
        }

        float[][] labelProbArray = new float[1][3];
        if (interpreter == null || inputBuffer == null) {
            return "ERROR WITH INTERPRETER OR INPUT ARRAY";
        } else {
            interpreter.run(inputBuffer, labelProbArray);
            int largestI = 0;
            for (int i = 1; i < 3; i++) {
                if (labelProbArray[0][i] > labelProbArray[0][largestI]) {
                    largestI = i;
                }
            }

            if (largestI == 0) {
                return "LEFT";
            } else if (largestI == 1) {
                return "CENTER";
            } else {
                return "RIGHT";
            }
        }
    }
}
