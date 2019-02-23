package com.android.jackapp.jusbcam;

import java.util.List;
import android.content.Context;
import android.hardware.usb.UsbDevice;
import com.android.jackapp.jusbcam.usb.DeviceFilter;
import com.android.jackapp.jusbcam.usb.USBMonitor;

import org.opencv.R;

public class RequestUsbPermission {
	
	private USBMonitor usbMonitor;
	private Context context;

	public RequestUsbPermission() {
		// TODO Auto-generated constructor stub
	}
	
	public RequestUsbPermission(Context ctx, USBMonitor usbMon) {
		// TODO Auto-generated constructor stub
		context = ctx;
		usbMonitor = usbMon;
	}
	
	public UsbDevice getDevice(){
		final List<DeviceFilter> filter = DeviceFilter.getDeviceFilters(context, R.xml.device_filter);

		List<UsbDevice> list = usbMonitor.getDeviceList(filter.get(0));
		if(list.size() == 0) {
			return null;
		}else {
			return list.get(0);
		}
	}
	
	public void requestPermission(UsbDevice item){
		usbMonitor.requestPermission(item);
	}
	
}
