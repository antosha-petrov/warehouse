using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class HistoryPage : ContentPage
{
	public HistoryPage()
	{
		InitializeComponent();
        BindingContext = new HistoryPageViewModel();
    }

    protected override void OnAppearing()
    {
        base.OnAppearing();

        if (BindingContext is HistoryPageViewModel viewModel)
        {
            viewModel.RefreshItems();
        }
    }
}