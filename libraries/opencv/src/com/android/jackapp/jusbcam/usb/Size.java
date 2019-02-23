package com.android.jackapp.jusbcam.usb;


import java.util.Locale;

import android.os.Parcel;
import android.os.Parcelable;

public class Size implements Parcelable {
	// 9999 is still image
	public int type;
	public int index;
	public int width;
	public int height;

	public Size(final int _type, final int _index, final int _width, final int _height) {
		type = _type;
		index = _index;
		width = _width;
		height = _height;
	}

	private Size(final Parcel source) {
		type = source.readInt();
		index = source.readInt();;
		width = source.readInt();;
		height = source.readInt();;
	}

	public Size set(final Size other) {
		if (other != null) {
			type = other.type;
			index = other.index;
			width = other.width;
			height = other.height;
		}
		return this;
	}

	@Override
	public int describeContents() {
		return 0;
	}

	@Override
	public void writeToParcel(final Parcel dest, final int flags) {
		dest.writeInt(type);
		dest.writeInt(index);
		dest.writeInt(width);
		dest.writeInt(height);
	}


	@Override
	public String toString() {
		return String.format(Locale.US, "Size(%dx%d,type:%d,index:%d)", width, height, type, index);
	}


	public static final Creator<Size> CREATOR = new Creator<Size>() {
		@Override
		public Size createFromParcel(final Parcel source) {
			return new Size(source);
		}
		@Override
		public Size[] newArray(final int size) {
			return new Size[size];
		}
	};
}
