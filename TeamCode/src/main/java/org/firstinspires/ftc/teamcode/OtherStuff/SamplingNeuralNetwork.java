package org.firstinspires.ftc.teamcode.OtherStuff;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.support.annotation.ColorInt;

import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

    public static Bitmap getBitmapFromVuforia(VuforiaLocalizer vuLocal) {
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuLocal.getFrameQueue().take();
        } catch (InterruptedException e) {
            return null;
        }

        com.vuforia.Image rgb = null;

        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        if (rgb == null) {
            return null;
        }

        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        return bm;
    }

    public static String runPrediction(Interpreter interpreter, Bitmap vuforiaImage) {
        ByteBuffer inputBuffer = null;
            // get input stream
//            InputStream ims = context.getAssets().open(assetImagePath);
            // load image as Drawable
        Bitmap image = Bitmap.createScaledBitmap(vuforiaImage, 80, 53, false);
        image = Bitmap.createBitmap(image, 0, 15, 80, 29);
        inputBuffer = ByteBuffer.allocateDirect(BYTE_SIZE_OF_FLOAT * image.getWidth() * image.getHeight());
        inputBuffer.order(ByteOrder.nativeOrder());
        for (int y = 0; y < image.getHeight(); y++) {
            for (int x = 0; x < image.getWidth(); x++) {
                @ColorInt int rgb = image.getPixel(x, y);
                inputBuffer.putFloat((65536 * (Color.red(rgb))) + (256 * (Color.blue(rgb))) + Color.green(rgb));
            }
        }

        float[][] labelProbArray = new float[1][3];
        if (interpreter == null) {
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
