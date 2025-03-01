package com.sprout.db_paisa;

import android.content.Context;
import android.view.View;
import android.view.ViewGroup;
import android.view.LayoutInflater;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import java.util.ArrayList;

public class MC_RecyclerViewAdapter extends RecyclerView.Adapter<MC_RecyclerViewAdapter.MyViewHolder>{
    Context context;
    ArrayList<DBPaisaScanResult> scanResults;
    public MC_RecyclerViewAdapter(Context context, ArrayList<DBPaisaScanResult> scanResults) {
        this.context = context;
        this.scanResults = scanResults;
    }

    @NonNull
    @Override
    public MC_RecyclerViewAdapter.MyViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        LayoutInflater inflater = LayoutInflater.from(context);
        View view = inflater.inflate(R.layout.recyclerviewrow, parent, false);
        return new MC_RecyclerViewAdapter.MyViewHolder(view);
    }

    @Override
    public void onBindViewHolder(@NonNull MC_RecyclerViewAdapter.MyViewHolder holder, int position) {
        DBPaisaScanResult scanResult = scanResults.get(position);
        holder.header.setText(scanResult.getDeviceType());
        holder.details.setText(scanResult.toString());

        holder.bind(scanResult);

        holder.itemView.setOnClickListener(v -> {
            boolean expanded = scanResult.isExpanded();

            scanResult.setExpanded(!expanded);
            notifyItemChanged(position);
        });
    }

    @Override
    public int getItemCount() {
        return scanResults.size();
    }

    public static class MyViewHolder extends RecyclerView.ViewHolder {

        TextView header;
        TextView details;

        public MyViewHolder(@NonNull View itemView) {
            super(itemView);

            header = itemView.findViewById(R.id.textView);
            details = itemView.findViewById(R.id.sub_item_details);
        }

        private void bind(DBPaisaScanResult scanResult) {
            boolean expanded = scanResult.isExpanded();

            details.setVisibility(expanded? View.VISIBLE : View.GONE);

            // do I have to set the values again
        }
    }
}
