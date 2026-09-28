package com.makeyourpet.chicaserver;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import com.makeyourpet.chicaserver.control.SurfaceStatus;
import java.util.Locale;

public class InfoSurfaceView extends SurfaceView implements SurfaceHolder.Callback {
    private final SurfaceHolder surfaceHolder;
    private Bitmap surfaceBitmap;
    private static final int[] LEG_DISPLAY_ORDER = {0, 3, 1, 4, 2, 5};
    private final Paint backgroundPaint = new Paint();
    private final Paint redPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint greenPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint yellowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint cyanPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint greyPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint bluePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint blackPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint strokeRedPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private SurfaceStatus status = new SurfaceStatus(Double.NaN, Double.NaN, 0.0d, "",
            0.0d, 0.0d, 0.0d, 0.0d, 0.0d, 0.0d, 0.0d, false, false,
            SurfaceStatus.ZONE_OK, SurfaceStatus.ZONE_OK, 0.0d);

    public InfoSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
        surfaceHolder = getHolder();
        surfaceHolder.addCallback(this);
        setBackgroundColor(0xff888888);
        setZOrderOnTop(true);
        backgroundPaint.setColor(0xff444444);
        configureTextPaint(redPaint, 0xffff0000);
        configureTextPaint(greenPaint, 0xff00ff00);
        configureTextPaint(yellowPaint, 0xffffff00);
        configureTextPaint(cyanPaint, 0xff00ffff);
        configureTextPaint(greyPaint, 0xffcccccc);
        configureTextPaint(bluePaint, 0xff0000ff);
        configureTextPaint(blackPaint, 0xff000000);
        strokeRedPaint.setColor(0xffff0000);
        strokeRedPaint.setStyle(Paint.Style.STROKE);
        strokeRedPaint.setStrokeWidth(5.0f);
    }

    public void setSurfaceStatus(SurfaceStatus status) {
        if (status != null) this.status = status;
        drawFrame();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        if (width <= 0 || height <= 0) return;
        surfaceBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        drawFrame();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        surfaceBitmap = null;
    }

    private void drawFrame() {
        Bitmap bmp = surfaceBitmap;
        if (bmp == null || bmp.isRecycled()) return;
        Canvas bitmapCanvas = new Canvas(bmp);
        drawStatus(bitmapCanvas, bmp.getWidth(), bmp.getHeight());
        Canvas canvas = null;
        try {
            canvas = surfaceHolder.lockCanvas();
            if (canvas != null) canvas.drawBitmap(bmp, 0.0f, 0.0f, null);
        } catch (Exception ignored) {
        } finally {
            if (canvas != null) {
                try {
                    surfaceHolder.unlockCanvasAndPost(canvas);
                } catch (Exception ignored) {
                }
            }
        }
    }

    private void drawStatus(Canvas canvas, int width, int height) {
        canvas.drawRect(0, 0, width, height, backgroundPaint);
        float textSize = getResources().getDimension(R.dimen.infoViewTextSize);
        float baseDimen = getResources().getDimension(R.dimen.baseInfoViewDimen);
        setTextSize(textSize);
        drawTopInfo(canvas, width);
        drawTouchBlocks(canvas, width, height, baseDimen);
        drawWarnings(canvas, width, height, baseDimen);
        drawPanels(canvas, width, height, baseDimen);
    }

    private void drawTopInfo(Canvas canvas, int width) {
        float labelX = width / 6.0f;
        float startY = labelX / 2.0f;
        float step = startY / 1.2f;
        drawInfoRow(canvas, "  V:", numberOrDashes(status.voltage), labelX, startY,
                zonePaint(status.voltageZone));
        drawInfoRow(canvas, "  I:", numberOrDashes(status.current), labelX, startY + step,
                zonePaint(status.currentZone));
        drawInfoRow(canvas, "BPS:", String.format(Locale.US, "% 3d", (int) status.bps),
                labelX, startY + (step * 2.0f), status.bps > 100.0d ? greenPaint : redPaint);
        drawInfoRow(canvas, " IP:", " " + status.ipAddress, labelX, startY + (step * 3.0f), greyPaint);
    }

    private void drawInfoRow(Canvas canvas, String label, String value,
            float labelX, float y, Paint valuePaint) {
        canvas.drawText(label, labelX, y, greyPaint);
        canvas.drawText(value, labelX * 2.0f, y, valuePaint);
    }

    // Immediate V/I value colour, matching the original z0.d (green when healthy,
    // yellow past the warn level, red past the cutoff level).
    private Paint zonePaint(int zone) {
        if (zone == SurfaceStatus.ZONE_CRITICAL) return redPaint;
        if (zone == SurfaceStatus.ZONE_WARN) return yellowPaint;
        return greenPaint;
    }

    private void drawTouchBlocks(Canvas canvas, int width, int height, float base) {
        float halfWidth = base;
        float halfHeight = 1.5f * base;
        for (int index : LEG_DISPLAY_ORDER) {
            float centerX = index <= 2 ? base : width - base;
            float centerY = (index == 1 || index == 4) ? height / 2.0f : halfHeight;
            if (index == 2 || index == 5) centerY = height - halfHeight;
            double touch = index < status.legTouches.length ? status.legTouches[index] : Double.NaN;
            Paint paint = !Double.isNaN(touch) && touch > 0.5d ? redPaint : blackPaint;
            canvas.drawRect((centerX - halfWidth) + 5.0f, (centerY - halfHeight) + 5.0f,
                    (centerX + halfWidth) - 5.0f, (centerY + halfHeight) - 5.0f, paint);
        }
    }

    private void drawWarnings(Canvas canvas, int width, int height, float base) {
        float warningHalf = base * 1.5f;
        float textOffset = base * 0.4f;
        if (status.voltageWarning) {
            float x = (width * 2.0f) / 5.0f;
            float y = height / 3.0f;
            canvas.drawRect(x - warningHalf, y - warningHalf, x + warningHalf, y + warningHalf, redPaint);
            canvas.drawText("V", x - textOffset, y + textOffset, yellowPaint);
        }
        if (status.currentWarning) {
            float x = (width * 3.0f) / 5.0f;
            float y = height / 3.0f;
            canvas.drawRect(x - warningHalf, y - warningHalf, x + warningHalf, y + warningHalf, redPaint);
            canvas.drawText("I", x - textOffset, y + textOffset, yellowPaint);
        }
    }

    private void drawPanels(Canvas canvas, int width, int height, float base) {
        float panelSize = width / 2.0f;
        float inset = 1.6f * base;
        float bottomPad = 3.0f * base;
        RectF right = new RectF(panelSize + inset, ((height - panelSize) - bottomPad) + inset,
                width - inset, (height - bottomPad) - inset);
        drawPanel(canvas, right);
        float rightRadius = ((right.right - right.left) / 2.0f) - inset;
        float rightCx = (right.left + right.right) / 2.0f;
        float rightCy = (right.top + right.bottom) / 2.0f;
        canvas.drawCircle(rightCx + ((float) status.strafe * rightRadius),
                rightCy + ((float) status.forward * rightRadius),
                base * 0.4f, greenPaint);
        double heading = status.turn;
        canvas.drawCircle(rightCx + (float) (Math.sin(heading) * rightRadius * 0.7f),
                rightCy - (float) (Math.cos(heading) * rightRadius * 0.7f),
                base * 0.28f, bluePaint);
        canvas.drawText(String.format(Locale.US, "% 5.2f", status.panelBps),
                rightCx - rightRadius, rightCy + rightRadius + 60.0f, greyPaint);

        RectF left = new RectF(inset, ((height - panelSize) - bottomPad) + inset,
                panelSize - inset, (height - bottomPad) - inset);
        drawPanel(canvas, left);
        float leftCx = (left.left + left.right) / 2.0f;
        float leftCy = (left.top + left.bottom) / 2.0f;
        canvas.drawCircle(((float) status.primaryX * 2.5f) + leftCx,
                ((float) status.primaryY * -2.5f) + leftCy,
                base * 0.4f, cyanPaint);
        canvas.drawCircle(((float) status.secondaryX * 10.0f) + leftCx,
                ((float) status.secondaryY * 10.0f) + leftCy,
                base * 0.4f, greenPaint);
    }

    private void drawPanel(Canvas canvas, RectF rect) {
        canvas.drawRect(rect, blackPaint);
        float cx = (rect.left + rect.right) / 2.0f;
        float cy = (rect.top + rect.bottom) / 2.0f;
        canvas.drawLine(rect.left, cy, rect.right, cy, greyPaint);
        canvas.drawLine(cx, rect.top, cx, rect.bottom, greyPaint);
    }

    private void setTextSize(float size) {
        redPaint.setTextSize(size);
        greenPaint.setTextSize(size);
        yellowPaint.setTextSize(size);
        cyanPaint.setTextSize(size);
        greyPaint.setTextSize(size);
        bluePaint.setTextSize(size);
        blackPaint.setTextSize(size);
    }

    private static void configureTextPaint(Paint paint, int color) {
        paint.setColor(color);
        paint.setTypeface(Typeface.create("serif-monospace", Typeface.BOLD));
        paint.setTextSize(48.0f);
    }

    private static String numberOrDashes(double value) {
        if (Double.isNaN(value)) return "---";
        return String.format(Locale.ENGLISH, "% 3.3f", value);
    }
}
