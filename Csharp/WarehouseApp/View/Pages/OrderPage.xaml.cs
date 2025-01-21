using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class OrderPage : ContentPage
{
	public OrderPage()
	{
		InitializeComponent();

        var mainPage = Application.Current?.Windows[0]?.Page as AppShell;

        if (mainPage != null &&
            mainPage.Items.FirstOrDefault() is FlyoutItem flyoutItem &&
            flyoutItem.Items.FirstOrDefault() is ShellSection shellSection &&
            shellSection.CurrentItem is ShellContent shellContent &&
            shellContent.Content is MainPage mainPageInstance &&
            mainPageInstance.BindingContext is MainPageViewModel mainPageViewModel)
        {
            var components = mainPageViewModel.GetComponents();
            BindingContext = new OrderPageViewModel(components);
        }
        else
        {
            throw new InvalidOperationException("Unable to locate MainPage or its BindingContext.");
        }
    }
}