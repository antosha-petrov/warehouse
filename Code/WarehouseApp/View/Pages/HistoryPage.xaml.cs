using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class HistoryPage : ContentPage
{
    public HistoryPage()
	{
		InitializeComponent();
        BindingContext = new HistoryPageViewModel();
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();

        if (BindingContext is HistoryPageViewModel viewModel)
        {
            await viewModel.RefreshItemsAsync();
        }
    }
}