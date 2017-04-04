package com.kunato.imagestitching;
import android.graphics.Color;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import org.w3c.dom.Text;

public class MainActivity extends AppCompatActivity {
    boolean mFirstTime = true;
    MainController mView;
    public Button b;
    TextView textView;
    boolean mRecording = false;
    private void initComponent(){
        b = new Button(this);
        LinearLayout linearLayout = new LinearLayout(this);
        textView = new TextView(this);
        b.setText("AE LOCK");
        textView.setText("Test");
        textView.setTextColor(Color.GREEN);

        linearLayout.addView(textView);
        linearLayout.addView(b);

        b.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                start();
            }
        });
        linearLayout.setGravity(Gravity.CENTER_HORIZONTAL | Gravity.BOTTOM);
        this.addContentView(linearLayout,
                new ViewGroup.LayoutParams(ViewGroup.LayoutParams.FILL_PARENT, ViewGroup.LayoutParams.FILL_PARENT));

    }
    public void start(){
        mView.runProcess(mFirstTime);
        if(mFirstTime){
            b.setText("Capture : 0");
            mFirstTime = false;
        }
        else{
            Log.d("Activity","Click");
            b.setBackgroundColor(Color.RED);
            mRecording = true;
        }
    }
    public void updateMenu(Menu menu){
        MenuItem item = menu.findItem(R.id.start);
        if(!mFirstTime){
            item.setIcon(R.drawable.ic_play_arrow_white_48dp);
        }
        else{
            item.setIcon(R.drawable.ic_center_focus_strong_white_48dp);
        }
        item = menu.findItem(R.id.location);
        if(!MainController.RESTORE_LOCATION){
            item.setTitle("Use Recorded location");
        }
        else{
            item.setTitle("Use Updated location");
        }
    }
    @Override
     public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu,menu);
        updateMenu(menu);
        return super.onCreateOptionsMenu(menu);
    }
    @Override
     public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()){
            case R.id.start:
                start();
                if(!mRecording){
                    item.setIcon(R.drawable.ic_play_arrow_white_48dp);
                }
                else{
                    item.setIcon(R.drawable.ic_stop_white_48dp);
                }
                break;
            case R.id.location:
                MainController.RESTORE_LOCATION = !MainController.RESTORE_LOCATION;
                Log.d("App","Location press");
                break;
            case R.id.data1:
                Log.d("App","Data1");
                mView.setDataSet(1);
                break;
            case R.id.data2:
                Log.d("App","Data2");
                mView.setDataSet(2);
                break;
            case R.id.data3:
                Log.d("App","Data3");
                mView.setDataSet(3);
                break;
            default:
                return super.onOptionsItemSelected(item);
        }
        return true;
    }

    public TextView getTextView() {
        return textView;
    }

    public Button getButton(){
        return b;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mView = new MainController(this);
        setContentView(mView);
        initComponent();
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if(!hasFocus) {
            Log.d("Activity", "Lost focus !");
            //       finishAndRemoveTask();

        }
    }

    @Override
    protected void onPause() {
        mView.Pause();
        mView.onPause();

        super.onPause();

    }

    @Override
    protected void onResume() {

        super.onResume();
        mView.onResume();
        //mView.Resume();
    }

}